/*
 * ***** BEGIN GPL LICENSE BLOCK *****
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * Contributor(s):
 *
 * ***** END GPL LICENSE BLOCK *****
 */

#include "MEM_guardedalloc.h"

#include "BLI_utildefines.h"
#include "BLI_alloca.h"
#include "BLI_memarena.h"
#include "BLI_math.h"
#include "BLI_rand.h"
#include "BLI_heap.h"
#include "BLI_boxpack2d.h"
#include "BLI_convexhull2d.h"

#include "blender_abf.h"

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "BLI_sys_types.h"  /* for intptr_t support */

#include "linear_solver.h"

/* Utils */

#if 0
#  define param_assert(condition)
#  define param_warning(message)
#  define param_test_equals_ptr(condition)
#  define param_test_equals_int(condition)
#else
#  define param_assert(condition) \
        if (!(condition)) \
            { /*printf("Assertion %s:%d\n", __FILE__, __LINE__); abort();*/ } (void)0
#  define param_warning(message) \
        { /*printf("Warning %s:%d: %s\n", __FILE__, __LINE__, message);*/ } (void)0
#  if 0
#    define param_test_equals_ptr(str, a, b) \
        if (a != b) \
            { /*printf("Equals %s => %p != %p\n", str, a, b);*/ } (void)0
#    define param_test_equals_int(str, a, b) \
        if (a != b) \
            { /*printf("Equals %s => %d != %d\n", str, a, b);*/ } (void)0
#  endif
#endif
typedef enum PBool {
    P_TRUE = 1,
    P_FALSE = 0
} PBool;

/* Special Purpose Hash */

typedef intptr_t PHashKey;

typedef struct PHashLink {
    struct PHashLink *next;
    PHashKey key;
} PHashLink;

typedef struct PHash {
    PHashLink **list;
    PHashLink **buckets;
    int size, cursize, cursize_id;
} PHash;



struct PVert;
struct PEdge;
struct PFace;
struct PChart;
struct PHandle;

/* Simplices */

typedef struct PVert {
    struct PVert *nextlink;

    union PVertUnion {
        PHashKey key;           /* construct */
        int id;                 /* abf/lscm matrix index */
        float distortion;       /* area smoothing */
        HeapNode *heaplink;     /* edge collapsing */
    } u;

    struct PEdge *edge;
    float co[3];
    float uv[2];
    unsigned char flag;

} PVert; 

typedef struct PEdge {
    struct PEdge *nextlink;

    union PEdgeUnion {
        PHashKey key;                   /* construct */
        int id;                         /* abf matrix index */
        HeapNode *heaplink;             /* fill holes */
        struct PEdge *nextcollapse;     /* simplification */
    } u;

    struct PVert *vert;
    struct PEdge *pair;
    struct PEdge *next;
    struct PFace *face;
    float *orig_uv, old_uv[2];
    unsigned short flag;

} PEdge;

typedef struct PFace {
    struct PFace *nextlink;

    union PFaceUnion {
        PHashKey key;           /* construct */
        int chart;              /* construct splitting*/
        float area3d;           /* stretch */
        int id;                 /* abf matrix index */
    } u;

    struct PEdge *edge;
    unsigned char flag;
} PFace;

enum PVertFlag {
    PVERT_PIN = 1,
    PVERT_SELECT = 2,
    PVERT_INTERIOR = 4,
    PVERT_COLLAPSE = 8,
    PVERT_SPLIT = 16
};

enum PEdgeFlag {
    PEDGE_SEAM = 1,
    PEDGE_VERTEX_SPLIT = 2,
    PEDGE_PIN = 4,
    PEDGE_SELECT = 8,
    PEDGE_DONE = 16,
    PEDGE_FILLED = 32,
    PEDGE_COLLAPSE = 64,
    PEDGE_COLLAPSE_EDGE = 128,
    PEDGE_COLLAPSE_PAIR = 256
};

/* for flipping faces */
#define PEDGE_VERTEX_FLAGS (PEDGE_PIN)

enum PFaceFlag {
    PFACE_CONNECTED = 1,
    PFACE_FILLED = 2,
    PFACE_COLLAPSE = 4
};

/* Chart */

typedef struct PChart {
    PVert *verts;
    PEdge *edges;
    PFace *faces;
    int nverts, nedges, nfaces;

    PVert *collapsed_verts;
    PEdge *collapsed_edges;
    PFace *collapsed_faces;

    union PChartUnion {
        struct PChartLscm {
            LinearSolver *context;
            float *abf_alpha;
            PVert *pin1, *pin2;
        } lscm;
        struct PChartPack {
            float rescale, area;
            float size[2] /* , trans[2] */;
        } pack;
    } u;

    unsigned char flag;
    struct PHandle *handle;
} PChart;

enum PChartFlag {
    PCHART_NOPACK = 1
};

enum PHandleState {
    PHANDLE_STATE_ALLOCATED,
    PHANDLE_STATE_CONSTRUCTED,
    PHANDLE_STATE_LSCM,
    PHANDLE_STATE_STRETCH
};

typedef struct PHandle {
    enum PHandleState state;
    MemArena *arena;

    PChart *construction_chart;
    PHash *hash_verts;
    PHash *hash_edges;
    PHash *hash_faces;

    PChart **charts;
    int ncharts;

    float aspx, aspy;

    RNG *rng;
    float blend;
    char do_aspect;
} PHandle;

/* PHash
 * - special purpose hash that keeps all its elements in a single linked list.
 * - after construction, this hash is thrown away, and the list remains.
 * - removing elements is not possible efficiently.
 */

static int PHashSizes[] = {
    1, 3, 5, 11, 17, 37, 67, 131, 257, 521, 1031, 2053, 4099, 8209, 
    16411, 32771, 65537, 131101, 262147, 524309, 1048583, 2097169, 
    4194319, 8388617, 16777259, 33554467, 67108879, 134217757, 268435459
};

#define PHASH_hash(ph, item) (((uintptr_t) (item)) % ((unsigned int) (ph)->cursize))
#define PHASH_edge(v1, v2)   (((v1) < (v2)) ? ((v1) * 39) ^ ((v2) * 31) : ((v1) * 31) ^ ((v2) * 39))

static PHash *phash_new(PHashLink **list, int sizehint)
{
    PHash *ph = (PHash *)MEM_callocN(sizeof(PHash), "PHash");
    ph->size = 0;
    ph->cursize_id = 0;
    ph->list = list;

    while (PHashSizes[ph->cursize_id] < sizehint)
        ph->cursize_id++;

    ph->cursize = PHashSizes[ph->cursize_id];
    ph->buckets = (PHashLink **)MEM_callocN(ph->cursize * sizeof(*ph->buckets), "PHashBuckets");

    return ph;
}

static void phash_delete(PHash *ph)
{
    MEM_freeN(ph->buckets);
    MEM_freeN(ph);
}

static int phash_size(PHash *ph)
{
    return ph->size;
}

static void phash_insert(PHash *ph, PHashLink *link)
{
    int size = ph->cursize;
    uintptr_t hash = PHASH_hash(ph, link->key);
    PHashLink *lookup = ph->buckets[hash];

    if (lookup == NULL) {
        /* insert in front of the list */
        ph->buckets[hash] = link;
        link->next = *(ph->list);
        *(ph->list) = link;
    }
    else {
        /* insert after existing element */
        link->next = lookup->next;
        lookup->next = link;
    }
        
    ph->size++;

    if (ph->size > (size * 3)) {
        PHashLink *next = NULL, *first = *(ph->list);

        ph->cursize = PHashSizes[++ph->cursize_id];
        MEM_freeN(ph->buckets);
        ph->buckets = (PHashLink **)MEM_callocN(ph->cursize * sizeof(*ph->buckets), "PHashBuckets");
        ph->size = 0;
        *(ph->list) = NULL;

        for (link = first; link; link = next) {
            next = link->next;
            phash_insert(ph, link);
        }
    }
}

static PHashLink *phash_lookup(PHash *ph, PHashKey key)
{
    PHashLink *link;
    uintptr_t hash = PHASH_hash(ph, key);

    for (link = ph->buckets[hash]; link; link = link->next)
        if (link->key == key)
            return link;
        else if (PHASH_hash(ph, link->key) != hash)
            return NULL;
    
    return link;
}

static PHashLink *phash_next(PHash *ph, PHashKey key, PHashLink *link)
{
    uintptr_t hash = PHASH_hash(ph, key);

    for (link = link->next; link; link = link->next)
        if (link->key == key)
            return link;
        else if (PHASH_hash(ph, link->key) != hash)
            return NULL;
    
    return link;
}

/* Geometry */

static float p_vec_angle_cos(float *v1, float *v2, float *v3)
{
    float d1[3], d2[3];

    d1[0] = v1[0] - v2[0];
    d1[1] = v1[1] - v2[1];
    d1[2] = v1[2] - v2[2];

    d2[0] = v3[0] - v2[0];
    d2[1] = v3[1] - v2[1];
    d2[2] = v3[2] - v2[2];

    normalize_v3(d1);
    normalize_v3(d2);

    return d1[0] * d2[0] + d1[1] * d2[1] + d1[2] * d2[2];
}

static float p_vec_angle(float *v1, float *v2, float *v3)
{
    float dot = p_vec_angle_cos(v1, v2, v3);

    if (dot <= -1.0f)
        return (float)M_PI;
    else if (dot >= 1.0f)
        return 0.0f;
    else
        return acosf(dot);
}

static float p_vec2_angle(float *v1, float *v2, float *v3)
{
    float u1[3], u2[3], u3[3];

    u1[0] = v1[0]; u1[1] = v1[1]; u1[2] = 0.0f;
    u2[0] = v2[0]; u2[1] = v2[1]; u2[2] = 0.0f;
    u3[0] = v3[0]; u3[1] = v3[1]; u3[2] = 0.0f;

    return p_vec_angle(u1, u2, u3);
}

static void p_triangle_angles(float *v1, float *v2, float *v3, float *a1, float *a2, float *a3)
{
    *a1 = p_vec_angle(v3, v1, v2);
    *a2 = p_vec_angle(v1, v2, v3);
    *a3 = (float)M_PI - *a2 - *a1;
}

static void p_face_angles(PFace *f, float *a1, float *a2, float *a3)
{
    PEdge *e1 = f->edge, *e2 = e1->next, *e3 = e2->next;
    PVert *v1 = e1->vert, *v2 = e2->vert, *v3 = e3->vert;

    p_triangle_angles(v1->co, v2->co, v3->co, a1, a2, a3);
}

static float p_face_area(PFace *f)
{
    PEdge *e1 = f->edge, *e2 = e1->next, *e3 = e2->next;
    PVert *v1 = e1->vert, *v2 = e2->vert, *v3 = e3->vert;

    return area_tri_v3(v1->co, v2->co, v3->co);
}

static float p_area_signed(float *v1, float *v2, float *v3)
{
    return 0.5f * (((v2[0] - v1[0]) * (v3[1] - v1[1])) -
                   ((v3[0] - v1[0]) * (v2[1] - v1[1])));
}

static float p_face_uv_area_signed(PFace *f)
{
    PEdge *e1 = f->edge, *e2 = e1->next, *e3 = e2->next;
    PVert *v1 = e1->vert, *v2 = e2->vert, *v3 = e3->vert;

    return 0.5f * (((v2->uv[0] - v1->uv[0]) * (v3->uv[1] - v1->uv[1])) -
                   ((v3->uv[0] - v1->uv[0]) * (v2->uv[1] - v1->uv[1])));
}

static float p_edge_length(PEdge *e)
{
    PVert *v1 = e->vert, *v2 = e->next->vert;
    float d[3];

    d[0] = v2->co[0] - v1->co[0];
    d[1] = v2->co[1] - v1->co[1];
    d[2] = v2->co[2] - v1->co[2];

    return sqrtf(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
}

static float p_edge_uv_length(PEdge *e)
{
    PVert *v1 = e->vert, *v2 = e->next->vert;
    float d[3];

    d[0] = v2->uv[0] - v1->uv[0];
    d[1] = v2->uv[1] - v1->uv[1];

    return sqrtf(d[0] * d[0] + d[1] * d[1]);
}

static void p_chart_uv_bbox(PChart *chart, float minv[2], float maxv[2])
{
    PVert *v;

    INIT_MINMAX2(minv, maxv);

    for (v = chart->verts; v; v = v->nextlink) {
        minmax_v2v2_v2(minv, maxv, v->uv);
    }
}

static void p_chart_uv_scale(PChart *chart, float scale)
{
    PVert *v;

    for (v = chart->verts; v; v = v->nextlink) {
        v->uv[0] *= scale;
        v->uv[1] *= scale;
    }
}

static void p_chart_uv_scale_xy(PChart *chart, float x, float y)
{
    PVert *v;

    for (v = chart->verts; v; v = v->nextlink) {
        v->uv[0] *= x;
        v->uv[1] *= y;
    }
}

static void p_chart_uv_translate(PChart *chart, float trans[2])
{
    PVert *v;

    for (v = chart->verts; v; v = v->nextlink) {
        v->uv[0] += trans[0];
        v->uv[1] += trans[1];
    }
}

static void p_chart_uv_transform(PChart *chart, float mat[2][2])
{
    PVert *v;

    for (v = chart->verts; v; v = v->nextlink) {
        mul_m2v2(mat, v->uv);
    }
}

static void p_chart_uv_to_array(PChart *chart, float (*points)[2])
{
    PVert *v;
    unsigned int i = 0;

    for (v = chart->verts; v; v = v->nextlink) {
        copy_v2_v2(points[i++], v->uv);
    }
}

static void UNUSED_FUNCTION(p_chart_uv_from_array)(PChart *chart, float (*points)[2])
{
    PVert *v;
    unsigned int i = 0;

    for (v = chart->verts; v; v = v->nextlink) {
        copy_v2_v2(v->uv, points[i++]);
    }
}


static PBool p_intersect_line_2d_dir(float *v1, float *dir1, float *v2, float *dir2, float *isect)
{
    float lmbda, div;

    div = dir2[0] * dir1[1] - dir2[1] * dir1[0];

    if (div == 0.0f)
        return P_FALSE;

    lmbda = ((v1[1] - v2[1]) * dir1[0] - (v1[0] - v2[0]) * dir1[1]) / div;
    isect[0] = v1[0] + lmbda * dir2[0];
    isect[1] = v1[1] + lmbda * dir2[1];

    return P_TRUE;
}

#if 0
static PBool p_intersect_line_2d(float *v1, float *v2, float *v3, float *v4, float *isect)
{
    float dir1[2], dir2[2];

    dir1[0] = v4[0] - v3[0];
    dir1[1] = v4[1] - v3[1];

    dir2[0] = v2[0] - v1[0];
    dir2[1] = v2[1] - v1[1];

    if (!p_intersect_line_2d_dir(v1, dir1, v2, dir2, isect)) {
        /* parallel - should never happen in theory for polygon kernel, but
         * let's give a point nearby in case things go wrong */
        isect[0] = (v1[0] + v2[0]) * 0.5f;
        isect[1] = (v1[1] + v2[1]) * 0.5f;
        return P_FALSE;
    }

    return P_TRUE;
}
#endif

/* Topological Utilities */

static PEdge *p_wheel_edge_next(PEdge *e)
{
    return e->next->next->pair;
}

static PEdge *p_wheel_edge_prev(PEdge *e)
{   
    return (e->pair) ? e->pair->next : NULL;
}

static PEdge *p_boundary_edge_next(PEdge *e)
{
    return e->next->vert->edge;
}

static PEdge *p_boundary_edge_prev(PEdge *e)
{
    PEdge *we = e, *last;

    do {
        last = we;
        we = p_wheel_edge_next(we);
    } while (we && (we != e));

    return last->next->next;
}

static PBool p_vert_interior(PVert *v)
{
    return (v->edge->pair != NULL);
}

static void p_face_flip(PFace *f)
{
    PEdge *e1 = f->edge, *e2 = e1->next, *e3 = e2->next;
    PVert *v1 = e1->vert, *v2 = e2->vert, *v3 = e3->vert;
    int f1 = e1->flag, f2 = e2->flag, f3 = e3->flag;
    float *orig_uv1 = e1->orig_uv, *orig_uv2 = e2->orig_uv, *orig_uv3 = e3->orig_uv;

    e1->vert = v2;
    e1->next = e3;
    e1->orig_uv = orig_uv2;
    e1->flag = (f1 & ~PEDGE_VERTEX_FLAGS) | (f2 & PEDGE_VERTEX_FLAGS);

    e2->vert = v3;
    e2->next = e1;
    e2->orig_uv = orig_uv3;
    e2->flag = (f2 & ~PEDGE_VERTEX_FLAGS) | (f3 & PEDGE_VERTEX_FLAGS);

    e3->vert = v1;
    e3->next = e2;
    e3->orig_uv = orig_uv1;
    e3->flag = (f3 & ~PEDGE_VERTEX_FLAGS) | (f1 & PEDGE_VERTEX_FLAGS);
}

#if 0
static void p_chart_topological_sanity_check(PChart *chart)
{
    PVert *v;
    PEdge *e;

    for (v = chart->verts; v; v = v->nextlink)
        param_test_equals_ptr("v->edge->vert", v, v->edge->vert);
    
    for (e = chart->edges; e; e = e->nextlink) {
        if (e->pair) {
            param_test_equals_ptr("e->pair->pair", e, e->pair->pair);
            param_test_equals_ptr("pair->vert", e->vert, e->pair->next->vert);
            param_test_equals_ptr("pair->next->vert", e->next->vert, e->pair->vert);
        }
    }
}
#endif

/* Loading / Flushing */

static void p_vert_load_pin_select_uvs(PHandle *handle, PVert *v)
{
    PEdge *e;
    int nedges = 0, npins = 0;
    float pinuv[2];

    v->uv[0] = v->uv[1] = 0.0f;
    pinuv[0] = pinuv[1] = 0.0f;
    e = v->edge;
    do {
        if (e->orig_uv) {
            if (e->flag & PEDGE_SELECT)
                v->flag |= PVERT_SELECT;

            if (e->flag & PEDGE_PIN) {
                pinuv[0] += e->orig_uv[0] * handle->aspx;
                pinuv[1] += e->orig_uv[1] * handle->aspy;
                npins++;
            }
            else {
                v->uv[0] += e->orig_uv[0] * handle->aspx;
                v->uv[1] += e->orig_uv[1] * handle->aspy;
            }

            nedges++;
        }

        e = p_wheel_edge_next(e);
    } while (e && e != (v->edge));

    if (npins > 0) {
        v->uv[0] = pinuv[0] / npins;
        v->uv[1] = pinuv[1] / npins;
        v->flag |= PVERT_PIN;
    }
    else if (nedges > 0) {
        v->uv[0] /= nedges;
        v->uv[1] /= nedges;
    }
}

static void p_flush_uvs(PHandle *handle, PChart *chart)
{
    PEdge *e;

    for (e = chart->edges; e; e = e->nextlink) {
        if (e->orig_uv) {
            e->orig_uv[0] = e->vert->uv[0] / handle->aspx;
            e->orig_uv[1] = e->vert->uv[1] / handle->aspy;
        }
    }
}

static void p_flush_uvs_blend(PHandle *handle, PChart *chart, float blend)
{
    PEdge *e;
    float invblend = 1.0f - blend;

    for (e = chart->edges; e; e = e->nextlink) {
        if (e->orig_uv) {
            e->orig_uv[0] = blend * e->old_uv[0] + invblend * e->vert->uv[0] / handle->aspx;
            e->orig_uv[1] = blend * e->old_uv[1] + invblend * e->vert->uv[1] / handle->aspy;
        }
    }
}

static void p_face_backup_uvs(PFace *f)
{
    PEdge *e1 = f->edge, *e2 = e1->next, *e3 = e2->next;

    if (e1->orig_uv) {
        e1->old_uv[0] = e1->orig_uv[0];
        e1->old_uv[1] = e1->orig_uv[1];
    }
    if (e2->orig_uv) {
        e2->old_uv[0] = e2->orig_uv[0];
        e2->old_uv[1] = e2->orig_uv[1];
    }
    if (e3->orig_uv) {
        e3->old_uv[0] = e3->orig_uv[0];
        e3->old_uv[1] = e3->orig_uv[1];
    }
}

static void p_face_restore_uvs(PFace *f)
{
    PEdge *e1 = f->edge, *e2 = e1->next, *e3 = e2->next;

    if (e1->orig_uv) {
        e1->orig_uv[0] = e1->old_uv[0];
        e1->orig_uv[1] = e1->old_uv[1];
    }
    if (e2->orig_uv) {
        e2->orig_uv[0] = e2->old_uv[0];
        e2->orig_uv[1] = e2->old_uv[1];
    }
    if (e3->orig_uv) {
        e3->orig_uv[0] = e3->old_uv[0];
        e3->orig_uv[1] = e3->old_uv[1];
    }
}

/* Construction (use only during construction, relies on u.key being set */

static PVert *p_vert_add(PHandle *handle, PHashKey key, const float co[3], PEdge *e)
{
    PVert *v = (PVert *)BLI_memarena_alloc(handle->arena, sizeof(*v));
    copy_v3_v3(v->co, co);
    v->u.key = key;
    v->edge = e;
    v->flag = 0;

    phash_insert(handle->hash_verts, (PHashLink *)v);

    return v;
}

static PVert *p_vert_lookup(PHandle *handle, PHashKey key, const float co[3], PEdge *e)
{
    PVert *v = (PVert *)phash_lookup(handle->hash_verts, key);

    if (v)
        return v;
    else
        return p_vert_add(handle, key, co, e);
}

static PVert *p_vert_copy(PChart *chart, PVert *v)
{
    PVert *nv = (PVert *)BLI_memarena_alloc(chart->handle->arena, sizeof(*nv));

    copy_v3_v3(nv->co, v->co);
    nv->uv[0] = v->uv[0];
    nv->uv[1] = v->uv[1];
    nv->u.key = v->u.key;
    nv->edge = v->edge;
    nv->flag = v->flag;

    return nv;
}

static PEdge *p_edge_lookup(PHandle *handle, PHashKey *vkeys)
{
    PHashKey key = PHASH_edge(vkeys[0], vkeys[1]);
    PEdge *e = (PEdge *)phash_lookup(handle->hash_edges, key);

    while (e) {
        if ((e->vert->u.key == vkeys[0]) && (e->next->vert->u.key == vkeys[1]))
            return e;
        else if ((e->vert->u.key == vkeys[1]) && (e->next->vert->u.key == vkeys[0]))
            return e;

        e = (PEdge *)phash_next(handle->hash_edges, key, (PHashLink *)e);
    }

    return NULL;
}

static int p_face_exists(ParamHandle *phandle, ParamKey *pvkeys, int i1, int i2, int i3)
{
    PHandle *handle = (PHandle *)phandle;
    PHashKey *vkeys = (PHashKey *)pvkeys;
    PHashKey key = PHASH_edge(vkeys[i1], vkeys[i2]);
    PEdge *e = (PEdge *)phash_lookup(handle->hash_edges, key);

    while (e) {
        if ((e->vert->u.key == vkeys[i1]) && (e->next->vert->u.key == vkeys[i2])) {
            if (e->next->next->vert->u.key == vkeys[i3])
                return P_TRUE;
        }
        else if ((e->vert->u.key == vkeys[i2]) && (e->next->vert->u.key == vkeys[i1])) {
            if (e->next->next->vert->u.key == vkeys[i3])
                return P_TRUE;
        }

        e = (PEdge *)phash_next(handle->hash_edges, key, (PHashLink *)e);
    }

    return P_FALSE;
}

static PChart *p_chart_new(PHandle *handle)
{
    PChart *chart = (PChart *)MEM_callocN(sizeof(*chart), "PChart");
    chart->handle = handle;

    return chart;
}

static void p_chart_delete(PChart *chart)
{
    /* the actual links are free by memarena */
    MEM_freeN(chart);
}

static PBool p_edge_implicit_seam(PEdge *e, PEdge *ep)
{
    float *uv1, *uv2, *uvp1, *uvp2;
    float limit[2];

    limit[0] = 0.00001;
    limit[1] = 0.00001;

    uv1 = e->orig_uv;
    uv2 = e->next->orig_uv;

    if (e->vert->u.key == ep->vert->u.key) {
        uvp1 = ep->orig_uv;
        uvp2 = ep->next->orig_uv;
    }
    else {
        uvp1 = ep->next->orig_uv;
        uvp2 = ep->orig_uv;
    }

    if ((fabsf(uv1[0] - uvp1[0]) > limit[0]) || (fabsf(uv1[1] - uvp1[1]) > limit[1])) {
        e->flag |= PEDGE_SEAM;
        ep->flag |= PEDGE_SEAM;
        return P_TRUE;
    }
    if ((fabsf(uv2[0] - uvp2[0]) > limit[0]) || (fabsf(uv2[1] - uvp2[1]) > limit[1])) {
        e->flag |= PEDGE_SEAM;
        ep->flag |= PEDGE_SEAM;
        return P_TRUE;
    }
    
    return P_FALSE;
}

static PBool p_edge_has_pair(PHandle *handle, PEdge *e, PEdge **pair, PBool impl)
{
    PHashKey key;
    PEdge *pe;
    PVert *v1, *v2;
    PHashKey key1 = e->vert->u.key;
    PHashKey key2 = e->next->vert->u.key;

    if (e->flag & PEDGE_SEAM)
        return P_FALSE;
    
    key = PHASH_edge(key1, key2);
    pe = (PEdge *)phash_lookup(handle->hash_edges, key);
    *pair = NULL;

    while (pe) {
        if (pe != e) {
            v1 = pe->vert;
            v2 = pe->next->vert;

            if (((v1->u.key == key1) && (v2->u.key == key2)) ||
                ((v1->u.key == key2) && (v2->u.key == key1)))
            {

                /* don't connect seams and t-junctions */
                if ((pe->flag & PEDGE_SEAM) || *pair ||
                    (impl && p_edge_implicit_seam(e, pe)))
                {
                    *pair = NULL;
                    return P_FALSE;
                }

                *pair = pe;
            }
        }

        pe = (PEdge *)phash_next(handle->hash_edges, key, (PHashLink *)pe);
    }

    if (*pair && (e->vert == (*pair)->vert)) {
        if ((*pair)->next->pair || (*pair)->next->next->pair) {
            /* non unfoldable, maybe mobius ring or klein bottle */
            *pair = NULL;
            return P_FALSE;
        }
    }

    return (*pair != NULL);
}

static PBool p_edge_connect_pair(PHandle *handle, PEdge *e, PEdge ***stack, PBool impl)
{
    PEdge *pair = NULL;

    if (!e->pair && p_edge_has_pair(handle, e, &pair, impl)) {
        if (e->vert == pair->vert)
            p_face_flip(pair->face);

        e->pair = pair;
        pair->pair = e;

        if (!(pair->face->flag & PFACE_CONNECTED)) {
            **stack = pair;
            (*stack)++;
        }
    }

    return (e->pair != NULL);
}

static int p_connect_pairs(PHandle *handle, PBool impl)
{
    PEdge **stackbase = MEM_mallocN(sizeof(*stackbase) * phash_size(handle->hash_faces), "Pstackbase");
    PEdge **stack = stackbase;
    PFace *f, *first;
    PEdge *e, *e1, *e2;
    PChart *chart = handle->construction_chart;
    int ncharts = 0;

    /* connect pairs, count edges, set vertex-edge pointer to a pairless edge */
    for (first = chart->faces; first; first = first->nextlink) {
        if (first->flag & PFACE_CONNECTED)
            continue;

        *stack = first->edge;
        stack++;

        while (stack != stackbase) {
            stack--;
            e = *stack;
            e1 = e->next;
            e2 = e1->next;

            f = e->face;
            f->flag |= PFACE_CONNECTED;

            /* assign verts to charts so we can sort them later */
            f->u.chart = ncharts;

            if (!p_edge_connect_pair(handle, e, &stack, impl))
                e->vert->edge = e;
            if (!p_edge_connect_pair(handle, e1, &stack, impl))
                e1->vert->edge = e1;
            if (!p_edge_connect_pair(handle, e2, &stack, impl))
                e2->vert->edge = e2;
        }

        ncharts++;
    }

    MEM_freeN(stackbase);

    return ncharts;
}

static void p_split_vert(PChart *chart, PEdge *e)
{
    PEdge *we, *lastwe = NULL;
    PVert *v = e->vert;
    PBool copy = P_TRUE;

    if (e->flag & PEDGE_VERTEX_SPLIT)
        return;

    /* rewind to start */
    lastwe = e;
    for (we = p_wheel_edge_prev(e); we && (we != e); we = p_wheel_edge_prev(we))
        lastwe = we;
    
    /* go over all edges in wheel */
    for (we = lastwe; we; we = p_wheel_edge_next(we)) {
        if (we->flag & PEDGE_VERTEX_SPLIT)
            break;

        we->flag |= PEDGE_VERTEX_SPLIT;

        if (we == v->edge) {
            /* found it, no need to copy */
            copy = P_FALSE;
            v->nextlink = chart->verts;
            chart->verts = v;
            chart->nverts++;
        }
    }

    if (copy) {
        /* not found, copying */
        v->flag |= PVERT_SPLIT;
        v = p_vert_copy(chart, v);
        v->flag |= PVERT_SPLIT;

        v->nextlink = chart->verts;
        chart->verts = v;
        chart->nverts++;

        v->edge = lastwe;

        we = lastwe;
        do {
            we->vert = v;
            we = p_wheel_edge_next(we);
        } while (we && (we != lastwe));
    }
}

static PChart **p_split_charts(PHandle *handle, PChart *chart, int ncharts)
{
    PChart **charts = MEM_mallocN(sizeof(*charts) * ncharts, "PCharts"), *nchart;
    PFace *f, *nextf;
    int i;

    for (i = 0; i < ncharts; i++)
        charts[i] = p_chart_new(handle);

    f = chart->faces;
    while (f) {
        PEdge *e1 = f->edge, *e2 = e1->next, *e3 = e2->next;
        nextf = f->nextlink;

        nchart = charts[f->u.chart];

        f->nextlink = nchart->faces;
        nchart->faces = f;
        e1->nextlink = nchart->edges;
        nchart->edges = e1;
        e2->nextlink = nchart->edges;
        nchart->edges = e2;
        e3->nextlink = nchart->edges;
        nchart->edges = e3;

        nchart->nfaces++;
        nchart->nedges += 3;

        p_split_vert(nchart, e1);
        p_split_vert(nchart, e2);
        p_split_vert(nchart, e3);

        f = nextf;
    }

    return charts;
}

static PFace *p_face_add(PHandle *handle)
{
    PFace *f;
    PEdge *e1, *e2, *e3;

    /* allocate */
    f = (PFace *)BLI_memarena_alloc(handle->arena, sizeof(*f));
    f->flag = 0;  /* init ! */

    e1 = (PEdge *)BLI_memarena_alloc(handle->arena, sizeof(*e1));
    e2 = (PEdge *)BLI_memarena_alloc(handle->arena, sizeof(*e2));
    e3 = (PEdge *)BLI_memarena_alloc(handle->arena, sizeof(*e3));

    /* set up edges */
    f->edge = e1;
    e1->face = e2->face = e3->face = f;

    e1->next = e2;
    e2->next = e3;
    e3->next = e1;

    e1->pair = NULL;
    e2->pair = NULL;
    e3->pair = NULL;

    e1->flag = 0;
    e2->flag = 0;
    e3->flag = 0;

    return f;
}

static PFace *p_face_add_construct(PHandle *handle, ParamKey key, ParamKey *vkeys,
                                   float *co[4], float *uv[4], int i1, int i2, int i3,
                                   ParamBool *pin, ParamBool *select)
{
    PFace *f = p_face_add(handle);
    PEdge *e1 = f->edge, *e2 = e1->next, *e3 = e2->next;

    e1->vert = p_vert_lookup(handle, vkeys[i1], co[i1], e1);
    e2->vert = p_vert_lookup(handle, vkeys[i2], co[i2], e2);
    e3->vert = p_vert_lookup(handle, vkeys[i3], co[i3], e3);

    e1->orig_uv = uv[i1];
    e2->orig_uv = uv[i2];
    e3->orig_uv = uv[i3];

    if (pin) {
        if (pin[i1]) e1->flag |= PEDGE_PIN;
        if (pin[i2]) e2->flag |= PEDGE_PIN;
        if (pin[i3]) e3->flag |= PEDGE_PIN;
    }

    if (select) {
        if (select[i1]) e1->flag |= PEDGE_SELECT;
        if (select[i2]) e2->flag |= PEDGE_SELECT;
        if (select[i3]) e3->flag |= PEDGE_SELECT;
    }

    /* insert into hash */
    f->u.key = key;
    phash_insert(handle->hash_faces, (PHashLink *)f);

    e1->u.key = PHASH_edge(vkeys[i1], vkeys[i2]);
    e2->u.key = PHASH_edge(vkeys[i2], vkeys[i3]);
    e3->u.key = PHASH_edge(vkeys[i3], vkeys[i1]);

    phash_insert(handle->hash_edges, (PHashLink *)e1);
    phash_insert(handle->hash_edges, (PHashLink *)e2);
    phash_insert(handle->hash_edges, (PHashLink *)e3);

    return f;
}

static PFace *p_face_add_fill(PChart *chart, PVert *v1, PVert *v2, PVert *v3)
{
    PFace *f = p_face_add(chart->handle);
    PEdge *e1 = f->edge, *e2 = e1->next, *e3 = e2->next;

    e1->vert = v1;
    e2->vert = v2;
    e3->vert = v3;

    e1->orig_uv = e2->orig_uv = e3->orig_uv = NULL;

    f->nextlink = chart->faces;
    chart->faces = f;
    e1->nextlink = chart->edges;
    chart->edges = e1;
    e2->nextlink = chart->edges;
    chart->edges = e2;
    e3->nextlink = chart->edges;
    chart->edges = e3;

    chart->nfaces++;
    chart->nedges += 3;

    return f;
}

static PBool p_quad_split_direction(PHandle *handle, float **co, PHashKey *vkeys)
{
    /* slight bias to prefer one edge over the other in case they are equal, so
     * that in symmetric models we choose the same split direction instead of
     * depending on floating point errors to decide */
    float bias = 1.0f + 1e-6f;
    float fac = len_v3v3(co[0], co[2]) * bias - len_v3v3(co[1], co[3]);
    PBool dir = (fac <= 0.0f);

    /* the face exists check is there because of a special case: when
     * two quads share three vertices, they can each be split into two
     * triangles, resulting in two identical triangles. for example in
     * suzanne's nose. */
    if (dir) {
        if (p_face_exists(handle, vkeys, 0, 1, 2) || p_face_exists(handle, vkeys, 0, 2, 3))
            return !dir;
    }
    else {
        if (p_face_exists(handle, vkeys, 0, 1, 3) || p_face_exists(handle, vkeys, 1, 2, 3))
            return !dir;
    }

    return dir;
}

/* Construction: boundary filling */

static void p_chart_boundaries(PChart *chart, int *nboundaries, PEdge **outer)
{   
    PEdge *e, *be;
    float len, maxlen = -1.0;

    if (nboundaries)
        *nboundaries = 0;
    if (outer)
        *outer = NULL;

    for (e = chart->edges; e; e = e->nextlink) {
        if (e->pair || (e->flag & PEDGE_DONE))
            continue;

        if (nboundaries)
            (*nboundaries)++;

        len = 0.0f;

        be = e;
        do {
            be->flag |= PEDGE_DONE;
            len += p_edge_length(be);
            be = be->next->vert->edge;
        } while (be != e);

        if (outer && (len > maxlen)) {
            *outer = e;
            maxlen = len;
        }
    }

    for (e = chart->edges; e; e = e->nextlink)
        e->flag &= ~PEDGE_DONE;
}

static float p_edge_boundary_angle(PEdge *e)
{
    PEdge *we;
    PVert *v, *v1, *v2;
    float angle;
    int n = 0;

    v = e->vert;

    /* concave angle check -- could be better */
    angle = M_PI;

    we = v->edge;
    do {
        v1 = we->next->vert;
        v2 = we->next->next->vert;
        angle -= p_vec_angle(v1->co, v->co, v2->co);

        we = we->next->next->pair;
        n++;
    } while (we && (we != v->edge));

    return angle;
}

static void p_chart_fill_boundary(PChart *chart, PEdge *be, int nedges)
{
    PEdge *e, *e1, *e2;

    PFace *f;
    struct Heap *heap = BLI_heap_new();
    float angle;

    e = be;
    do {
        angle = p_edge_boundary_angle(e);
        e->u.heaplink = BLI_heap_insert(heap, angle, e);

        e = p_boundary_edge_next(e);
    } while (e != be);

    if (nedges == 2) {
        /* no real boundary, but an isolated seam */
        e = be->next->vert->edge;
        e->pair = be;
        be->pair = e;

        BLI_heap_remove(heap, e->u.heaplink);
        BLI_heap_remove(heap, be->u.heaplink);
    }
    else {
        while (nedges > 2) {
            PEdge *ne, *ne1, *ne2;

            e = (PEdge *)BLI_heap_popmin(heap);

            e1 = p_boundary_edge_prev(e);
            e2 = p_boundary_edge_next(e);

            BLI_heap_remove(heap, e1->u.heaplink);
            BLI_heap_remove(heap, e2->u.heaplink);
            e->u.heaplink = e1->u.heaplink = e2->u.heaplink = NULL;

            e->flag |= PEDGE_FILLED;
            e1->flag |= PEDGE_FILLED;

            f = p_face_add_fill(chart, e->vert, e1->vert, e2->vert);
            f->flag |= PFACE_FILLED;

            ne = f->edge->next->next;
            ne1 = f->edge;
            ne2 = f->edge->next;

            ne->flag = ne1->flag = ne2->flag = PEDGE_FILLED;

            e->pair = ne;
            ne->pair = e;
            e1->pair = ne1;
            ne1->pair = e1;

            ne->vert = e2->vert;
            ne1->vert = e->vert;
            ne2->vert = e1->vert;

            if (nedges == 3) {
                e2->pair = ne2;
                ne2->pair = e2;
            }
            else {
                ne2->vert->edge = ne2;
                
                ne2->u.heaplink = BLI_heap_insert(heap, p_edge_boundary_angle(ne2), ne2);
                e2->u.heaplink = BLI_heap_insert(heap, p_edge_boundary_angle(e2), e2);
            }

            nedges--;
        }
    }

    BLI_heap_free(heap, NULL);
}

static void p_chart_fill_boundaries(PChart *chart, PEdge *outer)
{
    PEdge *e, *be; /* *enext - as yet unused */
    int nedges;

    for (e = chart->edges; e; e = e->nextlink) {
        /* enext = e->nextlink; - as yet unused */

        if (e->pair || (e->flag & PEDGE_FILLED))
            continue;

        nedges = 0;
        be = e;
        do {
            be->flag |= PEDGE_FILLED;
            be = be->next->vert->edge;
            nedges++;
        } while (be != e);

        if (e != outer)
            p_chart_fill_boundary(chart, e, nedges);
    }
}

/* ABF */

#define ABF_MAX_ITER 20

typedef struct PAbfSystem {
    int ninterior, nfaces, nangles;
    float *alpha, *beta, *sine, *cosine, *weight;
    float *bAlpha, *bTriangle, *bInterior;
    float *lambdaTriangle, *lambdaPlanar, *lambdaLength;
    float (*J2dt)[3], *bstar, *dstar;
    float minangle, maxangle;
} PAbfSystem;

static void p_abf_setup_system(PAbfSystem *sys)
{
    int i;

    sys->alpha = (float *)MEM_mallocN(sizeof(float) * sys->nangles, "ABFalpha");
    sys->beta = (float *)MEM_mallocN(sizeof(float) * sys->nangles, "ABFbeta");
    sys->sine = (float *)MEM_mallocN(sizeof(float) * sys->nangles, "ABFsine");
    sys->cosine = (float *)MEM_mallocN(sizeof(float) * sys->nangles, "ABFcosine");
    sys->weight = (float *)MEM_mallocN(sizeof(float) * sys->nangles, "ABFweight");

    sys->bAlpha = (float *)MEM_mallocN(sizeof(float) * sys->nangles, "ABFbalpha");
    sys->bTriangle = (float *)MEM_mallocN(sizeof(float) * sys->nfaces, "ABFbtriangle");
    sys->bInterior = (float *)MEM_mallocN(sizeof(float) * 2 * sys->ninterior, "ABFbinterior");

    sys->lambdaTriangle = (float *)MEM_callocN(sizeof(float) * sys->nfaces, "ABFlambdatri");
    sys->lambdaPlanar = (float *)MEM_callocN(sizeof(float) * sys->ninterior, "ABFlamdaplane");
    sys->lambdaLength = (float *)MEM_mallocN(sizeof(float) * sys->ninterior, "ABFlambdalen");

    sys->J2dt = MEM_mallocN(sizeof(float) * sys->nangles * 3, "ABFj2dt");
    sys->bstar = (float *)MEM_mallocN(sizeof(float) * sys->nfaces, "ABFbstar");
    sys->dstar = (float *)MEM_mallocN(sizeof(float) * sys->nfaces, "ABFdstar");

    for (i = 0; i < sys->ninterior; i++)
        sys->lambdaLength[i] = 1.0;
    
    sys->minangle = 1.0 * M_PI / 180.0;
    sys->maxangle = (float)M_PI - sys->minangle;
}

static void p_abf_free_system(PAbfSystem *sys)
{
    MEM_freeN(sys->alpha);
    MEM_freeN(sys->beta);
    MEM_freeN(sys->sine);
    MEM_freeN(sys->cosine);
    MEM_freeN(sys->weight);
    MEM_freeN(sys->bAlpha);
    MEM_freeN(sys->bTriangle);
    MEM_freeN(sys->bInterior);
    MEM_freeN(sys->lambdaTriangle);
    MEM_freeN(sys->lambdaPlanar);
    MEM_freeN(sys->lambdaLength);
    MEM_freeN(sys->J2dt);
    MEM_freeN(sys->bstar);
    MEM_freeN(sys->dstar);
}

static void p_abf_compute_sines(PAbfSystem *sys)
{
    int i;
    float *sine = sys->sine, *cosine = sys->cosine, *alpha = sys->alpha;

    for (i = 0; i < sys->nangles; i++, sine++, cosine++, alpha++) {
        *sine = sinf(*alpha);
        *cosine = cosf(*alpha);
    }
}

static float p_abf_compute_sin_product(PAbfSystem *sys, PVert *v, int aid)
{
    PEdge *e, *e1, *e2;
    float sin1, sin2;

    sin1 = sin2 = 1.0;

    e = v->edge;
    do {
        e1 = e->next;
        e2 = e->next->next;

        if (aid == e1->u.id) {
            /* we are computing a derivative for this angle,
             * so we use cos and drop the other part */
            sin1 *= sys->cosine[e1->u.id];
            sin2 = 0.0;
        }
        else
            sin1 *= sys->sine[e1->u.id];

        if (aid == e2->u.id) {
            /* see above */
            sin1 = 0.0;
            sin2 *= sys->cosine[e2->u.id];
        }
        else
            sin2 *= sys->sine[e2->u.id];

        e = e->next->next->pair;
    } while (e && (e != v->edge));

    return (sin1 - sin2);
}

static float p_abf_compute_grad_alpha(PAbfSystem *sys, PFace *f, PEdge *e)
{
    PVert *v = e->vert, *v1 = e->next->vert, *v2 = e->next->next->vert;
    float deriv;

    deriv = (sys->alpha[e->u.id] - sys->beta[e->u.id]) * sys->weight[e->u.id];
    deriv += sys->lambdaTriangle[f->u.id];

    if (v->flag & PVERT_INTERIOR) {
        deriv += sys->lambdaPlanar[v->u.id];
    }

    if (v1->flag & PVERT_INTERIOR) {
        float product = p_abf_compute_sin_product(sys, v1, e->u.id);
        deriv += sys->lambdaLength[v1->u.id] * product;
    }

    if (v2->flag & PVERT_INTERIOR) {
        float product = p_abf_compute_sin_product(sys, v2, e->u.id);
        deriv += sys->lambdaLength[v2->u.id] * product;
    }

    return deriv;
}

static float p_abf_compute_gradient(PAbfSystem *sys, PChart *chart)
{
    PFace *f;
    PEdge *e;
    PVert *v;
    float norm = 0.0;

    for (f = chart->faces; f; f = f->nextlink) {
        PEdge *e1 = f->edge, *e2 = e1->next, *e3 = e2->next;
        float gtriangle, galpha1, galpha2, galpha3;

        galpha1 = p_abf_compute_grad_alpha(sys, f, e1);
        galpha2 = p_abf_compute_grad_alpha(sys, f, e2);
        galpha3 = p_abf_compute_grad_alpha(sys, f, e3);

        sys->bAlpha[e1->u.id] = -galpha1;
        sys->bAlpha[e2->u.id] = -galpha2;
        sys->bAlpha[e3->u.id] = -galpha3;

        norm += galpha1 * galpha1 + galpha2 * galpha2 + galpha3 * galpha3;

        gtriangle = sys->alpha[e1->u.id] + sys->alpha[e2->u.id] + sys->alpha[e3->u.id] - (float)M_PI;
        sys->bTriangle[f->u.id] = -gtriangle;
        norm += gtriangle * gtriangle;
    }

    for (v = chart->verts; v; v = v->nextlink) {
        if (v->flag & PVERT_INTERIOR) {
            float gplanar = -2 * M_PI, glength;

            e = v->edge;
            do {
                gplanar += sys->alpha[e->u.id];
                e = e->next->next->pair;
            } while (e && (e != v->edge));

            sys->bInterior[v->u.id] = -gplanar;
            norm += gplanar * gplanar;

            glength = p_abf_compute_sin_product(sys, v, -1);
            sys->bInterior[sys->ninterior + v->u.id] = -glength;
            norm += glength * glength;
        }
    }

    return norm;
}

static PBool p_abf_matrix_invert(PAbfSystem *sys, PChart *chart)
{
    PFace *f;
    PEdge *e;
    int i, j, ninterior = sys->ninterior, nvar = 2 * sys->ninterior;
    PBool success;
    LinearSolver *context;

    context = EIG_linear_solver_new(0, nvar, 1);

    for (i = 0; i < nvar; i++)
        EIG_linear_solver_right_hand_side_add(context, 0, i, sys->bInterior[i]);

    for (f = chart->faces; f; f = f->nextlink) {
        float wi1, wi2, wi3, b, si, beta[3], j2[3][3], W[3][3];
        float row1[6], row2[6], row3[6];
        int vid[6];
        PEdge *e1 = f->edge, *e2 = e1->next, *e3 = e2->next;
        PVert *v1 = e1->vert, *v2 = e2->vert, *v3 = e3->vert;

        wi1 = 1.0f / sys->weight[e1->u.id];
        wi2 = 1.0f / sys->weight[e2->u.id];
        wi3 = 1.0f / sys->weight[e3->u.id];

        /* bstar1 = (J1*dInv*bAlpha - bTriangle) */
        b = sys->bAlpha[e1->u.id] * wi1;
        b += sys->bAlpha[e2->u.id] * wi2;
        b += sys->bAlpha[e3->u.id] * wi3;
        b -= sys->bTriangle[f->u.id];

        /* si = J1*d*J1t */
        si = 1.0f / (wi1 + wi2 + wi3);

        /* J1t*si*bstar1 - bAlpha */
        beta[0] = b * si - sys->bAlpha[e1->u.id];
        beta[1] = b * si - sys->bAlpha[e2->u.id];
        beta[2] = b * si - sys->bAlpha[e3->u.id];

        /* use this later for computing other lambda's */
        sys->bstar[f->u.id] = b;
        sys->dstar[f->u.id] = si;

        /* set matrix */
        W[0][0] = si - sys->weight[e1->u.id]; W[0][1] = si; W[0][2] = si;
        W[1][0] = si; W[1][1] = si - sys->weight[e2->u.id]; W[1][2] = si;
        W[2][0] = si; W[2][1] = si; W[2][2] = si - sys->weight[e3->u.id];

        vid[0] = vid[1] = vid[2] = vid[3] = vid[4] = vid[5] = -1;

        if (v1->flag & PVERT_INTERIOR) {
            vid[0] = v1->u.id;
            vid[3] = ninterior + v1->u.id;

            sys->J2dt[e1->u.id][0] = j2[0][0] = 1.0f * wi1;
            sys->J2dt[e2->u.id][0] = j2[1][0] = p_abf_compute_sin_product(sys, v1, e2->u.id) * wi2;
            sys->J2dt[e3->u.id][0] = j2[2][0] = p_abf_compute_sin_product(sys, v1, e3->u.id) * wi3;

            EIG_linear_solver_right_hand_side_add(context, 0, v1->u.id, j2[0][0] * beta[0]);
            EIG_linear_solver_right_hand_side_add(context, 0, ninterior + v1->u.id, j2[1][0] * beta[1] + j2[2][0] * beta[2]);

            row1[0] = j2[0][0] * W[0][0];
            row2[0] = j2[0][0] * W[1][0];
            row3[0] = j2[0][0] * W[2][0];

            row1[3] = j2[1][0] * W[0][1] + j2[2][0] * W[0][2];
            row2[3] = j2[1][0] * W[1][1] + j2[2][0] * W[1][2];
            row3[3] = j2[1][0] * W[2][1] + j2[2][0] * W[2][2];
        }

        if (v2->flag & PVERT_INTERIOR) {
            vid[1] = v2->u.id;
            vid[4] = ninterior + v2->u.id;

            sys->J2dt[e1->u.id][1] = j2[0][1] = p_abf_compute_sin_product(sys, v2, e1->u.id) * wi1;
            sys->J2dt[e2->u.id][1] = j2[1][1] = 1.0f * wi2;
            sys->J2dt[e3->u.id][1] = j2[2][1] = p_abf_compute_sin_product(sys, v2, e3->u.id) * wi3;

            EIG_linear_solver_right_hand_side_add(context, 0, v2->u.id, j2[1][1] * beta[1]);
            EIG_linear_solver_right_hand_side_add(context, 0, ninterior + v2->u.id, j2[0][1] * beta[0] + j2[2][1] * beta[2]);

            row1[1] = j2[1][1] * W[0][1];
            row2[1] = j2[1][1] * W[1][1];
            row3[1] = j2[1][1] * W[2][1];

            row1[4] = j2[0][1] * W[0][0] + j2[2][1] * W[0][2];
            row2[4] = j2[0][1] * W[1][0] + j2[2][1] * W[1][2];
            row3[4] = j2[0][1] * W[2][0] + j2[2][1] * W[2][2];
        }

        if (v3->flag & PVERT_INTERIOR) {
            vid[2] = v3->u.id;
            vid[5] = ninterior + v3->u.id;

            sys->J2dt[e1->u.id][2] = j2[0][2] = p_abf_compute_sin_product(sys, v3, e1->u.id) * wi1;
            sys->J2dt[e2->u.id][2] = j2[1][2] = p_abf_compute_sin_product(sys, v3, e2->u.id) * wi2;
            sys->J2dt[e3->u.id][2] = j2[2][2] = 1.0f * wi3;

            EIG_linear_solver_right_hand_side_add(context, 0, v3->u.id, j2[2][2] * beta[2]);
            EIG_linear_solver_right_hand_side_add(context, 0, ninterior + v3->u.id, j2[0][2] * beta[0] + j2[1][2] * beta[1]);

            row1[2] = j2[2][2] * W[0][2];
            row2[2] = j2[2][2] * W[1][2];
            row3[2] = j2[2][2] * W[2][2];

            row1[5] = j2[0][2] * W[0][0] + j2[1][2] * W[0][1];
            row2[5] = j2[0][2] * W[1][0] + j2[1][2] * W[1][1];
            row3[5] = j2[0][2] * W[2][0] + j2[1][2] * W[2][1];
        }

        for (i = 0; i < 3; i++) {
            int r = vid[i];

            if (r == -1)
                continue;

            for (j = 0; j < 6; j++) {
                int c = vid[j];

                if (c == -1)
                    continue;

                if (i == 0)
                    EIG_linear_solver_matrix_add(context, r, c, j2[0][i] * row1[j]);
                else
                    EIG_linear_solver_matrix_add(context, r + ninterior, c, j2[0][i] * row1[j]);

                if (i == 1)
                    EIG_linear_solver_matrix_add(context, r, c, j2[1][i] * row2[j]);
                else
                    EIG_linear_solver_matrix_add(context, r + ninterior, c, j2[1][i] * row2[j]);


                if (i == 2)
                    EIG_linear_solver_matrix_add(context, r, c, j2[2][i] * row3[j]);
                else
                    EIG_linear_solver_matrix_add(context, r + ninterior, c, j2[2][i] * row3[j]);
            }
        }
    }

    success = EIG_linear_solver_solve(context);

    if (success) {
        for (f = chart->faces; f; f = f->nextlink) {
            float dlambda1, pre[3], dalpha;
            PEdge *e1 = f->edge, *e2 = e1->next, *e3 = e2->next;
            PVert *v1 = e1->vert, *v2 = e2->vert, *v3 = e3->vert;

            pre[0] = pre[1] = pre[2] = 0.0;

            if (v1->flag & PVERT_INTERIOR) {
                float x = EIG_linear_solver_variable_get(context, 0, v1->u.id);
                float x2 = EIG_linear_solver_variable_get(context, 0, ninterior + v1->u.id);
                pre[0] += sys->J2dt[e1->u.id][0] * x;
                pre[1] += sys->J2dt[e2->u.id][0] * x2;
                pre[2] += sys->J2dt[e3->u.id][0] * x2;
            }

            if (v2->flag & PVERT_INTERIOR) {
                float x = EIG_linear_solver_variable_get(context, 0, v2->u.id);
                float x2 = EIG_linear_solver_variable_get(context, 0, ninterior + v2->u.id);
                pre[0] += sys->J2dt[e1->u.id][1] * x2;
                pre[1] += sys->J2dt[e2->u.id][1] * x;
                pre[2] += sys->J2dt[e3->u.id][1] * x2;
            }

            if (v3->flag & PVERT_INTERIOR) {
                float x = EIG_linear_solver_variable_get(context, 0, v3->u.id);
                float x2 = EIG_linear_solver_variable_get(context, 0, ninterior + v3->u.id);
                pre[0] += sys->J2dt[e1->u.id][2] * x2;
                pre[1] += sys->J2dt[e2->u.id][2] * x2;
                pre[2] += sys->J2dt[e3->u.id][2] * x;
            }

            dlambda1 = pre[0] + pre[1] + pre[2];
            dlambda1 = sys->dstar[f->u.id] * (sys->bstar[f->u.id] - dlambda1);
            
            sys->lambdaTriangle[f->u.id] += dlambda1;

            dalpha = (sys->bAlpha[e1->u.id] - dlambda1);
            sys->alpha[e1->u.id] += dalpha / sys->weight[e1->u.id] - pre[0];

            dalpha = (sys->bAlpha[e2->u.id] - dlambda1);
            sys->alpha[e2->u.id] += dalpha / sys->weight[e2->u.id] - pre[1];

            dalpha = (sys->bAlpha[e3->u.id] - dlambda1);
            sys->alpha[e3->u.id] += dalpha / sys->weight[e3->u.id] - pre[2];

            /* clamp */
            e = f->edge;
            do {
                if (sys->alpha[e->u.id] > (float)M_PI)
                    sys->alpha[e->u.id] = (float)M_PI;
                else if (sys->alpha[e->u.id] < 0.0f)
                    sys->alpha[e->u.id] = 0.0f;
            } while (e != f->edge);
        }

        for (i = 0; i < ninterior; i++) {
            sys->lambdaPlanar[i] += (float)EIG_linear_solver_variable_get(context, 0, i);
            sys->lambdaLength[i] += (float)EIG_linear_solver_variable_get(context, 0, ninterior + i);
        }
    }

    EIG_linear_solver_delete(context);

    return success;
}

static PBool p_chart_abf_solve(PChart *chart)
{
    PVert *v;
    PFace *f;
    PEdge *e, *e1, *e2, *e3;
    PAbfSystem sys;
    int i;
    float /* lastnorm, */ /* UNUSED */ limit = (chart->nfaces > 100) ? 1.0f : 0.001f;

    /* setup id's */
    sys.ninterior = sys.nfaces = sys.nangles = 0;

    for (v = chart->verts; v; v = v->nextlink) {
        if (p_vert_interior(v)) {
            v->flag |= PVERT_INTERIOR;
            v->u.id = sys.ninterior++;
        }
        else
            v->flag &= ~PVERT_INTERIOR;
    }

    for (f = chart->faces; f; f = f->nextlink) {
        e1 = f->edge; e2 = e1->next; e3 = e2->next;
        f->u.id = sys.nfaces++;

        /* angle id's are conveniently stored in half edges */
        e1->u.id = sys.nangles++;
        e2->u.id = sys.nangles++;
        e3->u.id = sys.nangles++;
    }

    p_abf_setup_system(&sys);

    /* compute initial angles */
    for (f = chart->faces; f; f = f->nextlink) {
        float a1, a2, a3;

        e1 = f->edge; e2 = e1->next; e3 = e2->next;
        p_face_angles(f, &a1, &a2, &a3);

        if (a1 < sys.minangle)
            a1 = sys.minangle;
        else if (a1 > sys.maxangle)
            a1 = sys.maxangle;
        if (a2 < sys.minangle)
            a2 = sys.minangle;
        else if (a2 > sys.maxangle)
            a2 = sys.maxangle;
        if (a3 < sys.minangle)
            a3 = sys.minangle;
        else if (a3 > sys.maxangle)
            a3 = sys.maxangle;

        sys.alpha[e1->u.id] = sys.beta[e1->u.id] = a1;
        sys.alpha[e2->u.id] = sys.beta[e2->u.id] = a2;
        sys.alpha[e3->u.id] = sys.beta[e3->u.id] = a3;

        sys.weight[e1->u.id] = 2.0f / (a1 * a1);
        sys.weight[e2->u.id] = 2.0f / (a2 * a2);
        sys.weight[e3->u.id] = 2.0f / (a3 * a3);
    }

    for (v = chart->verts; v; v = v->nextlink) {
        if (v->flag & PVERT_INTERIOR) {
            float anglesum = 0.0, scale;

            e = v->edge;
            do {
                anglesum += sys.beta[e->u.id];
                e = e->next->next->pair;
            } while (e && (e != v->edge));

            scale = (anglesum == 0.0f) ? 0.0f : 2.0f * (float)M_PI / anglesum;

            e = v->edge;
            do {
                sys.beta[e->u.id] = sys.alpha[e->u.id] = sys.beta[e->u.id] * scale;
                e = e->next->next->pair;
            } while (e && (e != v->edge));
        }
    }

    if (sys.ninterior > 0) {
        p_abf_compute_sines(&sys);

        /* iteration */
        /* lastnorm = 1e10; */ /* UNUSED */

        for (i = 0; i < ABF_MAX_ITER; i++) {
            float norm = p_abf_compute_gradient(&sys, chart);

            /* lastnorm = norm; */ /* UNUSED */

            if (norm < limit)
                break;

            if (!p_abf_matrix_invert(&sys, chart)) {
                param_warning("ABF failed to invert matrix");
                p_abf_free_system(&sys);
                return P_FALSE;
            }

            p_abf_compute_sines(&sys);
        }

        if (i == ABF_MAX_ITER) {
            param_warning("ABF maximum iterations reached");
            p_abf_free_system(&sys);
            return P_FALSE;
        }
    }

    chart->u.lscm.abf_alpha = MEM_dupallocN(sys.alpha);
    p_abf_free_system(&sys);

    return P_TRUE;
}