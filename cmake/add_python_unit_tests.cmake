
set(python_test_scripts
  arm_to_joint_target.py
  arm_to_cart_target.py
  fullbody_plan.py
  position_base.py
  this_side_up.py
)

foreach(pyfile ${python_test_scripts})
  message("adding test with path ${CMAKE_SOURCE_DIR}/python_examples/${pyfile}")
  message("python exc ${PYTHON_EXECUTABLE}")
  add_test(${pyfile} "/usr/bin/python" "${CMAKE_SOURCE_DIR}/python_examples/${pyfile}")
endforeach()

add_test(arm_to_cart_target_position_only "/usr/bin/python" "${CMAKE_SOURCE_DIR}/python_examples/arm_to_cart_target.py" "--position_only")

