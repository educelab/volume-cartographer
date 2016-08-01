import bpy
import csv

mesh = bpy.context.object.data
uv_layer = mesh.uv_layers.active.data

uvRaw = []

for poly in mesh.polygons:
    face = []
    for loop_index in poly.loop_indices:
        face.append(mesh.loops[loop_index].vertex_index)
        face.append(uv_layer[loop_index].uv[0])
        face.append(uv_layer[loop_index].uv[1])
    uvRaw.append(face)

with open('uvMap.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerows(uvRaw)
