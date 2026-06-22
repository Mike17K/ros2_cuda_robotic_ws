cmake_minimum_required(VERSION 3.8)
project(drone_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# --- ΠΡΟΣΘΕΣΕ ΑΥΤΟ ΤΟ ΚΟΜΜΑΤΙ ---
# Εγκατάσταση των φακέλων ώστε να είναι ορατοί στο share directory
install(
  DIRECTORY
    launch
    description  # Ή όπως αλλιώς λες τον φάκελο με τα xacro σου
    config       # Αν έχεις αρχεία παραμέτρων .yaml
    meshes       # Αν έχεις αρχεία .stl ή .dae για το drone
    description/components  # Αν έχεις αρχεία xacro για τα επιμέρους μέρη
  DESTINATION share/${PROJECT_NAME}
)
# --------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
