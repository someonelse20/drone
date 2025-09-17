file(REMOVE_RECURSE
  "libfusion.a"
  "libfusion.pdb"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/fusion.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
