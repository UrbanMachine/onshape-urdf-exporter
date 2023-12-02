import re
from pathlib import Path

import numpy as np
import stl
from stl import mesh


def load_mesh(stl_file):
    return mesh.Mesh.from_file(stl_file)


def save_mesh(mesh, stl_file):
    mesh.save(stl_file, mode=stl.Mode.BINARY)


def combine_meshes(m1, m2):
    return mesh.Mesh(np.concatenate([m1.data, m2.data]))


def apply_matrix(mesh, matrix):
    rotation = matrix[0:3, 0:3]
    translation = matrix[0:3, 3:4].T.tolist()

    def transform(points):
        return (rotation * np.matrix(points).T).T + translation * len(points)

    mesh.v0 = transform(mesh.v0)
    mesh.v1 = transform(mesh.v1)
    mesh.v2 = transform(mesh.v2)
    mesh.normals = transform(mesh.normals)


# Script taken from doing the needed operation
# (Filters > Remeshing, Simplification and Reconstruction >
# Quadric Edge Collapse Decimation, with parameters:
# 0.9 percentage reduction (10%), 0.3 Quality threshold (70%)
# Target number of faces is ignored with those parameters
# conserving face normals, planar simplification and
# post-simplimfication cleaning)
# And going to Filter > Show current filter script
filter_script_mlx = """
<!DOCTYPE FilterScript>
<FilterScript>
<filter name="Simplification: Quadric Edge Collapse Decimation">

<Param type="RichFloat" value="%reduction%" name="TargetPerc"/>
<Param type="RichFloat" value="0.5" name="QualityThr"/>
<Param type="RichBool" value="false" name="PreserveBoundary"/>
<Param type="RichFloat" value="1" name="BoundaryWeight"/>
<Param type="RichBool" value="true" name="PreserveNormal"/>
<Param type="RichBool" value="false" name="PreserveTopology"/>
<Param type="RichBool" value="true" name="OptimalPlacement"/>
<Param type="RichBool" value="true" name="PlanarQuadric"/>
<Param type="RichBool" value="false" name="QualityWeight"/>
<Param type="RichFloat" value="0.001" name="PlanarWeight"/>
<Param type="RichBool" value="true" name="AutoClean"/>
<Param type="RichBool" value="false" name="Selected"/>


</filter>
</FilterScript>
"""


def simplify_stl(path: Path) -> None:
    """Optimize a single STL file in-place, if it isn't already optimized."""

    import open3d as o3d

    # Check if this file has already been optimized
    stl_header = path.read_bytes()
    matches = list(re.finditer(b"Open3D", stl_header))
    if len(matches):
        print(f"Skipping {str(path)}, already optimized")
        return

    print(f"Optimizing {str(path)}")
    simple_mesh: o3d.geometry.TriangleMesh = o3d.io.read_triangle_mesh(str(path))
    simple_mesh = simple_mesh.compute_triangle_normals()
    simple_mesh = simple_mesh.simplify_vertex_clustering(
        voxel_size=0.0025, contraction=o3d.geometry.SimplificationContraction.Quadric
    )
    simple_mesh = simple_mesh.merge_close_vertices(eps=0.001)
    simple_mesh = simple_mesh.remove_duplicated_vertices()
    simple_mesh = simple_mesh.remove_duplicated_triangles()
    simple_mesh = simple_mesh.remove_degenerate_triangles()
    simple_mesh = simple_mesh.remove_non_manifold_edges()
    simple_mesh = simple_mesh.compute_triangle_normals()
    simple_mesh = simple_mesh.compute_vertex_normals()

    assert o3d.io.write_triangle_mesh(
        str(path),
        simple_mesh,
        write_vertex_normals=False,
        write_triangle_uvs=False,
        compressed=False,
        write_vertex_colors=False,
    )
