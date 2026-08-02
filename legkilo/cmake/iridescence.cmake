set(OpenGL_GL_PREFERENCE GLVND)

find_package(OpenGL REQUIRED)
find_package(glm REQUIRED)
find_package(glfw3 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PNG)
find_package(JPEG)

################
## Data paths ##
################

# Source and thirdparty roots inside this package
set(IRIDESCENCE_SRC_DIR "${PROJECT_SOURCE_DIR}/thirdparty/iridescence")
set(IRIDESCENCE_TP_DIR  "${PROJECT_SOURCE_DIR}/thirdparty/iridescence/thirdparty")
set(IRIDESCENCE_DATA_DIR "${IRIDESCENCE_SRC_DIR}/data")

###############
## Libraries ##
###############

set(EXTRA_LIBRARIES)
set(EXTRA_SOURCE)

if(MSVC)
  add_compile_definitions(NOMINMAX)
  add_compile_definitions(_USE_MATH_DEFINES)
  add_compile_options(/wd4018 /wd4244 /wd4267 /wd4305 /wd4312)
  add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/source-charset:utf-8>")
  # Enable parallel compilation
  add_compile_options(/MP)
else()
  list(APPEND EXTRA_LIBRARIES dl)
  list(APPEND EXTRA_LIBRARIES pthread)
endif()

if(assimp_FOUND)
  list(APPEND EXTRA_LIBRARIES assimp)
  list(APPEND EXTRA_SOURCE
    ${IRIDESCENCE_SRC_DIR}/glk/io/mesh_io.cpp
  )
endif()

if(PNG_FOUND)
  list(APPEND EXTRA_LIBRARIES PNG::PNG)
  list(APPEND EXTRA_SOURCE ${IRIDESCENCE_SRC_DIR}/glk/io/png_io.cpp)
else()
  list(APPEND EXTRA_SOURCE ${IRIDESCENCE_SRC_DIR}/glk/io/png_io_dummy.cpp)
endif()

if(JPEG_FOUND)
  list(APPEND EXTRA_LIBRARIES JPEG::JPEG)
  list(APPEND EXTRA_SOURCE ${IRIDESCENCE_SRC_DIR}/glk/io/jpeg_io.cpp)
else()
  list(APPEND EXTRA_SOURCE ${IRIDESCENCE_SRC_DIR}/glk/io/jpeg_io_dummy.cpp)
endif()

add_library(iridescence
  # GL and IMGUI
  ${IRIDESCENCE_TP_DIR}/gl3w/gl3w.cpp
  ${IRIDESCENCE_TP_DIR}/imgui/imgui.cpp
  ${IRIDESCENCE_TP_DIR}/imgui/imgui_demo.cpp
  ${IRIDESCENCE_TP_DIR}/imgui/imgui_draw.cpp
  ${IRIDESCENCE_TP_DIR}/imgui/imgui_tables.cpp
  ${IRIDESCENCE_TP_DIR}/imgui/imgui_widgets.cpp
  ${IRIDESCENCE_TP_DIR}/ImGuizmo/ImCurveEdit.cpp
  ${IRIDESCENCE_TP_DIR}/ImGuizmo/ImGradient.cpp
  ${IRIDESCENCE_TP_DIR}/ImGuizmo/ImGuizmo.cpp
  ${IRIDESCENCE_TP_DIR}/ImGuizmo/ImSequencer.cpp
  ${IRIDESCENCE_TP_DIR}/implot/implot.cpp
  ${IRIDESCENCE_TP_DIR}/implot/implot_demo.cpp
  ${IRIDESCENCE_TP_DIR}/implot/implot_items.cpp
  ${IRIDESCENCE_TP_DIR}/imgui/backends/imgui_impl_glfw.cpp
  ${IRIDESCENCE_TP_DIR}/imgui/backends/imgui_impl_opengl3.cpp

  # glk
  ${IRIDESCENCE_SRC_DIR}/glk/path_std.cpp
  ${IRIDESCENCE_SRC_DIR}/glk/hash.cpp
  ${IRIDESCENCE_SRC_DIR}/glk/mesh.cpp
  ${IRIDESCENCE_SRC_DIR}/glk/mesh_model.cpp
  ${IRIDESCENCE_SRC_DIR}/glk/lines.cpp
  ${IRIDESCENCE_SRC_DIR}/glk/thin_lines.cpp
  ${IRIDESCENCE_SRC_DIR}/glk/trajectory.cpp
  ${IRIDESCENCE_SRC_DIR}/glk/gridmap.cpp
  ${IRIDESCENCE_SRC_DIR}/glk/drawable_container.cpp
  ${IRIDESCENCE_SRC_DIR}/glk/async_buffer_copy.cpp
  ${IRIDESCENCE_SRC_DIR}/glk/pointcloud_buffer.cpp
  ${IRIDESCENCE_SRC_DIR}/glk/pointnormals_buffer.cpp
  ${IRIDESCENCE_SRC_DIR}/glk/point_correspondences.cpp
  ${IRIDESCENCE_SRC_DIR}/glk/normal_distributions.cpp
  ${IRIDESCENCE_SRC_DIR}/glk/indexed_pointcloud_buffer.cpp
  ${IRIDESCENCE_SRC_DIR}/glk/splatting.cpp
  ${IRIDESCENCE_SRC_DIR}/glk/colormap.cpp
  ${IRIDESCENCE_SRC_DIR}/glk/html_colors.cpp
  ${IRIDESCENCE_SRC_DIR}/glk/texture.cpp
  ${IRIDESCENCE_SRC_DIR}/glk/glsl_shader.cpp
  ${IRIDESCENCE_SRC_DIR}/glk/frame_buffer.cpp
  ${IRIDESCENCE_SRC_DIR}/glk/pixel_buffer.cpp
  ${IRIDESCENCE_SRC_DIR}/glk/shader_storage_buffer.cpp
  ${IRIDESCENCE_SRC_DIR}/glk/query.cpp
  ${IRIDESCENCE_SRC_DIR}/glk/debug_output.cpp
  ${IRIDESCENCE_SRC_DIR}/glk/transform_feedback.cpp
  ${IRIDESCENCE_SRC_DIR}/glk/texture_renderer.cpp
  ${IRIDESCENCE_SRC_DIR}/glk/primitives/primitives.cpp
  ${IRIDESCENCE_SRC_DIR}/glk/io/ascii_io.cpp
  ${IRIDESCENCE_SRC_DIR}/glk/io/ply_io.cpp
  ${IRIDESCENCE_SRC_DIR}/glk/io/image_io.cpp
  ${IRIDESCENCE_SRC_DIR}/glk/effects/plain_rendering.cpp
  ${IRIDESCENCE_SRC_DIR}/glk/effects/screen_space_splatting.cpp
  ${IRIDESCENCE_SRC_DIR}/glk/effects/screen_space_lighting.cpp
  ${IRIDESCENCE_SRC_DIR}/glk/effects/screen_space_ambient_occlusion.cpp
  ${IRIDESCENCE_SRC_DIR}/glk/effects/screen_space_attribute_estimation.cpp
  ${IRIDESCENCE_SRC_DIR}/glk/effects/naive_screen_space_ambient_occlusion.cpp

  # guik
  ${IRIDESCENCE_SRC_DIR}/guik/gl_canvas.cpp
  ${IRIDESCENCE_SRC_DIR}/guik/model_control.cpp
  ${IRIDESCENCE_SRC_DIR}/guik/hovered_drawings.cpp
  ${IRIDESCENCE_SRC_DIR}/guik/hovered_primitives.cpp
  ${IRIDESCENCE_SRC_DIR}/guik/imgui_application.cpp
  ${IRIDESCENCE_SRC_DIR}/guik/screen_capture.cpp
  ${IRIDESCENCE_SRC_DIR}/guik/recent_files.cpp
  ${IRIDESCENCE_SRC_DIR}/guik/camera/camera_control.cpp
  ${IRIDESCENCE_SRC_DIR}/guik/camera/orbit_camera_control_xy.cpp
  ${IRIDESCENCE_SRC_DIR}/guik/camera/orbit_camera_control_xz.cpp
  ${IRIDESCENCE_SRC_DIR}/guik/camera/topdown_camera_control.cpp
  ${IRIDESCENCE_SRC_DIR}/guik/camera/arcball_camera_control.cpp
  ${IRIDESCENCE_SRC_DIR}/guik/camera/static_camera_control.cpp
  ${IRIDESCENCE_SRC_DIR}/guik/camera/fps_camera_control.cpp
  ${IRIDESCENCE_SRC_DIR}/guik/camera/projection_control.cpp
  ${IRIDESCENCE_SRC_DIR}/guik/camera/basic_projection_control.cpp
  ${IRIDESCENCE_SRC_DIR}/guik/viewer/plot_data.cpp
  ${IRIDESCENCE_SRC_DIR}/guik/viewer/shader_setting.cpp
  ${IRIDESCENCE_SRC_DIR}/guik/viewer/light_viewer.cpp
  ${IRIDESCENCE_SRC_DIR}/guik/viewer/light_viewer_context.cpp
  ${IRIDESCENCE_SRC_DIR}/guik/viewer/light_viewer_context_util.cpp
  ${IRIDESCENCE_SRC_DIR}/guik/viewer/async_light_viewer.cpp
  ${IRIDESCENCE_SRC_DIR}/guik/viewer/async_light_viewer_context.cpp
  ${IRIDESCENCE_SRC_DIR}/guik/viewer/viewer_ui.cpp
  ${IRIDESCENCE_SRC_DIR}/guik/viewer/info_window.cpp
  ${IRIDESCENCE_SRC_DIR}/guik/viewer/anonymous.cpp

  ${EXTRA_SOURCE}
)

target_include_directories(iridescence PUBLIC
  $<BUILD_INTERFACE:${IRIDESCENCE_SRC_DIR}>
  $<BUILD_INTERFACE:${IRIDESCENCE_TP_DIR}/imgui>
  $<BUILD_INTERFACE:${IRIDESCENCE_TP_DIR}/imgui/backends>
  $<BUILD_INTERFACE:${IRIDESCENCE_TP_DIR}/ImGuizmo>
  $<BUILD_INTERFACE:${IRIDESCENCE_TP_DIR}/implot>
  $<BUILD_INTERFACE:${IRIDESCENCE_TP_DIR}/portable-file-dialogs>
  $<BUILD_INTERFACE:${IRIDESCENCE_TP_DIR}/gl3w>
  $<BUILD_INTERFACE:${IRIDESCENCE_TP_DIR}/rapidhash>
)

target_link_libraries(iridescence PUBLIC
  Eigen3::Eigen
  OpenGL::GL
  glfw
  ${EXTRA_LIBRARIES}
)

if(TARGET glm::glm)
  target_link_libraries(iridescence PRIVATE glm::glm)
elseif(GLM_INCLUDE_DIRS)
  target_include_directories(iridescence PRIVATE ${GLM_INCLUDE_DIRS})
endif()

target_compile_definitions(iridescence PRIVATE
  IMGUI_IMPL_OPENGL_LOADER_GL3W
  GL3W_EXPORTS
  GLK_EXPORTS
  DATA_PATH_GUESS="${IRIDESCENCE_DATA_DIR}"
)

set_target_properties(iridescence PROPERTIES POSITION_INDEPENDENT_CODE ON)
