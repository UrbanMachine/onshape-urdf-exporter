from pathlib import Path

import open3d as o3d
from colorama import Fore, Style


def simplify_stl(path: Path) -> None:
    """Optimize a single STL file in-place, if it isn't already optimized."""

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

    success = o3d.io.write_triangle_mesh(
        str(path),
        simple_mesh,
        write_vertex_normals=False,
        write_triangle_uvs=False,
        compressed=False,
        write_vertex_colors=False,
    )
    if not success:
        print(f"{Fore.YELLOW}WARNING: Failed to simplify STL {path}{Style.RESET_ALL}")
