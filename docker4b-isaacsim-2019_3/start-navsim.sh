~/isaac/packages/navsim/unity$ ./navsim.x86_64 --scene warehouse
bazel run packages/navsim/apps:navsim_viewer_tcp
bazel run //apps/carter/navsim:navsim_navigate -- --more packages/navsim/maps/warehouse.json,packages/navsim/robots/carter.json
