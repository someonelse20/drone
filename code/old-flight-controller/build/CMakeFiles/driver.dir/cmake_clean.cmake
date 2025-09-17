file(REMOVE_RECURSE
  "libdriver.a"
  "libdriver.pdb"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/driver.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
