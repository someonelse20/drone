file(REMOVE_RECURSE
  "libregisters.a"
  "libregisters.pdb"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/registers.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
