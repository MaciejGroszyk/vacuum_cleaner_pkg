import meshlib.mrmeshpy as mr
# load raster image:
dm = mr.loadDistanceMapFromImage(mr.Path("/home/maciej/vc_ws/src/vacuum_cleaner_pkg/random/random_map_rev.png"), 0)
# find the boundary contour between black and white:
polyline2 = mr.distanceMapTo2DIsoPolyline(dm, isoValue=127)
# compute the triangulation inside the contour
mesh = mr.triangulateContours(polyline2.contours2())
# save 2D triangulation in a textual OBJ file:
mr.saveMesh(mesh, mr.Path("/home/maciej/vc_ws/src/vacuum_cleaner_pkg/random/random_map_rev.stl"))