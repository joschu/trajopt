
set(python_test_scripts
  arm_to_cart_target.py
  arm_to_joint_target.py
  position_base.py
)
foreach(pyfile ${python_test_scripts})
  message("adding test with path ${CMAKE_SOURCE_DIR}/python_examples/${pyfile}")
  message("python exc ${PYTHON_EXECUTABLE}")
  add_test(${pyfile} "/usr/bin/python" "${CMAKE_SOURCE_DIR}/python_examples/${pyfile}")
endforeach()