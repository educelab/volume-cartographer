#pragma once

/** @file */

#include <cmath>
#include <iostream>

#include <BulletSoftBody/btSoftBody.h>
#include <BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <btBulletDynamicsCommon.h>

#include "vc/core/types/ITKMesh.hpp"
#include "vc/core/types/UVMap.hpp"

namespace volcart
{
namespace experimental
{
namespace texturing
{

/** Forward the constrain motion callback @memberof ClothModelingUVMapping */
static void constrainMotionCallback(btDynamicsWorld* world, btScalar timeStep);

/** Forward the axis lock callback @memberof ClothModelingUVMapping */
static void axisLockCallback(btDynamicsWorld* world, btScalar timeStep);

/** Forward the "move to target" callback @memberof ClothModelingUVMapping */
static void moveTowardTargetCallback(btDynamicsWorld* world, btScalar timeStep);

/** Forward the "do nothing" callback @memberof ClothModelingUVMapping */
static void emptyPreTickCallback(btDynamicsWorld* world, btScalar timeStep);

/**
 * @class ClothModelingUVMapping
 * @author Abigail Coleman, Seth Parker
 * @date 3/14/16
 *
 * @brief Computes a 2D parameterization of a triangular mesh using a cloth
 * modeling simulation
 *
 * The mesh being flattened can be thought of as a "wrinkled sheet" in 3D space.
 * First, two corners of the sheet are pinned in place and the rest of the
 * sheet is allowed to unfurl relative to these points. Second, the sheet is
 * dropped on a collision plane that is roughly parallel to the sheet surface.
 * This helps to smooth out many of the largely curved sections and puts most
 * surface points on the same place. Finally, the boundary of the sheet is
 * stretched "outward" (along the surface plane) and then allowed to relax to a
 * resting position that minimizes the error in the surface area of the sheet.
 *
 * The results of this method are highly dependent on the input mesh and the
 * large number of parameters that can be tweaked. There's probably not much use
 * use for this class as is, but it's a decent reference for how to use the
 * Bullet Physics library. The general framework could be useful for improving
 * the results of parameterizations generated by other methods.
 *
 * @ingroup UV
 */
class ClothModelingUVMapping
{
public:
    /** @brief List of vertex IDs */
    using VertIDList = std::vector<uint64_t>;

    /** Enumeration of the algorithm stages */
    enum Stage { Unfurl, Collision, Expansion };

    /** @brief Pinned vertex class */
    struct Pin {
        /** Vertex index */
        uint64_t index;
        /** Pointer to vertex in soft body */
        btSoftBody::Node* node;
        /** Target position */
        btVector3 target;
    };

    /**@{*/
    /** @brief Construct with input parameters
     *
     * @param input Input mesh
     * @param unfurlIterations No. of iterations in the unfurl stage
     * @param collideIterations No. of iterations in the collision stage
     * @param expandIterations No. of iterations in the expansion stage
     * @param unfurlPins Pinned vertices during the unfurl stage
     * @param expansionPins Pinned vertices during the expansion stage
     */
    ClothModelingUVMapping(
        ITKMesh::Pointer input,
        uint16_t unfurlIterations,
        uint16_t collideIterations,
        uint16_t expandIterations,
        VertIDList unfurlPins,
        VertIDList expansionPins);

    /** @brief Default destructor */
    ~ClothModelingUVMapping();
    /**@}*/

    /**@{*/
    /** @brief Get the flattened surface as a mesh
     *
     * @warning The vertices of this mesh are in simulation world coordinates,
     * not volume coordinates.
     */
    ITKMesh::Pointer getMesh();

    /** @brief Get the flattened surface as a UV map */
    volcart::UVMap getUVMap();
    /**@}*/

    /**@{*/
    /** @brief Set the acceleration for a stage in the process
     *
     * The acceleration is the "gravity" force in the simulation world. During
     * the unfurl stage, the acceleration acts along the X axis to roughly
     * flatten the surface relative to the pinned vertices. In the collision and
     * expansion stages, it is the force applied along the Y axis that collides
     * the mesh against the collision plane.
     */
    void setAcceleration(Stage s, double a);
    /**@}*/

    /**@{*/
    /** @brief Compute the parameterization */
    UVMap compute();

    /** @brief Run the unfurl stage */
    void unfurl();

    /** @brief Run the collision stage */
    void collide();

    /** @brief Run the expansion stage */
    void expand();
    /**@}*/

    /**@{*/
    /**
     * CALLBACK: Limits vertex movement to only the X axis by setting the other
     * components to 0. Only used in the unfurl stage.
     *
     * @warning This function is a pretick callback and only has effect when
     * running the simulation.
     */
    void cbConstrainMotion(btScalar timeStep);

    /**
     * CALLBACK: Applies a corrective acceleration to points that have passed
     * through the collision plane.
     *
     * @warning This function is a pretick callback and only has effect when
     * running the simulation.
     */
    void cbAxisLock(btScalar timeStep);

    /**
     * CALLBACK: Moves pinned vertices towards their target position. Only
     * used by the expansion stage.
     *
     * @warning This function is a pretick callback and only has effect when
     * running the simulation.
     */
    void cbMoveTowardTarget(btScalar timeStep);

    /**
     * CALLBACK: Has no effect. Used to clear the simulation of any previously
     * assigend callbacks.
     *
     * @warning This function is a pretick callback and only has effect when
     * running the simulation.
     */
    void cbEmptyPreTick(btScalar timeStep);
    /**@}*/

private:
    /** Input mesh */
    const ITKMesh::Pointer mesh_;
    /** SoftBody version of input mesh */
    btSoftBody* softBody_;
    /** Scale factor between the mesh and simulation world */
    double meshToWorldScale_;
    /** Surface Area of the mesh before being flattened */
    double startingSurfaceArea_;
    /** List of pins for the current simulation */
    std::vector<Pin> currentPins_;

    /** No. of iterations in the unfurl stage */
    uint16_t unfurlIterations_;
    /** Unfurl stage acceleration. Default: 10 */
    double unfurlA_;
    /** Pinned vertices for the unfurl stage */
    VertIDList unfurlPins_;

    /** Run the unfurl stage */
    void unfurl_();

    /** No. of iterations in the collision stage */
    uint16_t collideIterations_;
    /** Collision stage acceleration. Default: -10 */
    double collisionA_;
    /** Collision plane rigid body */
    btRigidBody* collisionPlane_;

    /** Run the collision stage */
    void collide_();

    /** No. of iterations in the expansion stage */
    uint16_t expandIterations_;
    /** Expansion stage acceleration. Default: 10 */
    double expansionA_;
    /** Pinned vertices for the expansion stage */
    VertIDList expansionPins_;

    /** Run the expansion stage */
    void expand_();

    /** The same as volcart::meshmath::SurfaceArea(), adapted for soft bodies */
    double surface_area_();

    /** Bullet physics collision detection interface */
    btBroadphaseInterface* worldBroadphase_;
    /** Bullet physics collision configuration */
    btDefaultCollisionConfiguration* worldCollisionConfig_;
    /** Bullet physics collision handler */
    btCollisionDispatcher* worldCollisionDispatcher_;
    /** Bullet physics simulation world solver */
    btSequentialImpulseConstraintSolver* worldConstraintSolver_;
    /** Bullet physics soft body solver */
    btSoftBodySolver* worldSoftBodySolver_;
    /** Bullet physics simulation world */
    btSoftRigidDynamicsWorld* world_;
};
}  // namespace texturing
}  // namespace experimental
}  // namespace volcart
