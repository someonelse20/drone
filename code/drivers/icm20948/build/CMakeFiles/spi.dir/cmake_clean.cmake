file(REMOVE_RECURSE
  "libspi.a"
  "libspi.pdb"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/spi.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
