nanobind_add_module(groundgrid MODULE NOMINSIZE groundgrid_ext.cpp)
target_link_libraries(groundgrid PRIVATE groundgrid_lib)
install(TARGETS groundgrid LIBRARY DESTINATION .)