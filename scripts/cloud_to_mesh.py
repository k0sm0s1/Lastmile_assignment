#!/usr/bin/env python3
"""
Stage 2a: turn the cleaned, floor-aligned point cloud into a mesh Gazebo can
use as a static collision + visual object.

The source cloud has no faces (0 triangles) - it's a raw scan, not a mesh -
so this is a real reconstruction step, not a format conversion. Approach:

  1. Re-estimate normals consistently (the shipped normals are per-point from
     the original SLAM pipeline and aren't guaranteed to be globally
     consistent, which Poisson reconstruction needs).
  2. Poisson surface reconstruction (depth=9) to get a watertight-ish mesh.
  3. Trim low-density vertices - Poisson extrapolates a closed surface even
     through the holes left by a walked-path scan (doorways, unscanned gaps),
     so low-density regions are exactly where it's hallucinating rather than
     reconstructing. We cut those out with the density values Poisson itself
     reports for each vertex.
  4. Decimate to a physics-friendly triangle budget (real-time Gazebo physics
     chokes on multi-million-triangle collision meshes).
  5. Export .obj (visual) and a copy for the collision geometry.

Output: src/lastmile_description/models/scanned_environment/meshes/environment.obj
"""
import sys
import numpy as np
import open3d as o3d

IN_CLOUD = "data/cleaned_cloud.ply"
OUT_MESH = "src/lastmile_description/models/scanned_environment/meshes/environment.obj"

POISSON_DEPTH = 9
DENSITY_TRIM_QUANTILE = 0.03   # drop the lowest-density 3% of vertices (hole hallucination)
TARGET_TRIANGLES = 150_000     # decimation target: enough detail for collision, still real-time


def main():
    print(f"Loading {IN_CLOUD} ...")
    pcd = o3d.io.read_point_cloud(IN_CLOUD)
    print(f"  {len(pcd.points):,} points")

    print("Estimating consistent normals ...")
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    pcd.orient_normals_consistent_tangent_plane(30)

    print(f"Poisson reconstruction (depth={POISSON_DEPTH}) ...")
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=POISSON_DEPTH)
    densities = np.asarray(densities)
    print(f"  raw mesh: {len(mesh.vertices):,} verts / {len(mesh.triangles):,} tris")

    thresh = np.quantile(densities, DENSITY_TRIM_QUANTILE)
    verts_to_remove = densities < thresh
    mesh.remove_vertices_by_mask(verts_to_remove)
    print(f"  trimmed low-density (hallucinated) regions: {len(mesh.vertices):,} verts / "
          f"{len(mesh.triangles):,} tris")

    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_duplicated_vertices()
    mesh.remove_non_manifold_edges()

    if len(mesh.triangles) > TARGET_TRIANGLES:
        print(f"Decimating to ~{TARGET_TRIANGLES:,} triangles ...")
        mesh = mesh.simplify_quadric_decimation(TARGET_TRIANGLES)
        print(f"  decimated: {len(mesh.vertices):,} verts / {len(mesh.triangles):,} tris")

    mesh.compute_vertex_normals()

    o3d.io.write_triangle_mesh(OUT_MESH, mesh)
    print(f"Wrote {OUT_MESH}")

    aabb = mesh.get_axis_aligned_bounding_box()
    print(f"Mesh bounds: min={aabb.min_bound}, max={aabb.max_bound}")


if __name__ == "__main__":
    sys.exit(main())
