# OnShape URDF Exporter

Exports OnShape assemblies into URDF files with STL meshes. This is based off
of the URDF exporting functionality provided by onshape-to-robot, but with
some improvements:

- XML document creation is done using Python's XML library, fixing a number of
  bugs related to characters not being properly escaped
- Files created by this tool always have valid filenames, even on Windows
- Uses Open3D for STL simplification instead of MeshLab
