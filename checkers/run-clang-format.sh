#!/bin/bash
# Find all the files in this project
CPP_FILES=$(find ../ | grep -e '\.c"\?$' -e '\.cpp"\?$' -e '\.h"\?$' -e '\.hpp"\?$')

# Format all the files
clang-format -i $CPP_FILES -style=Google
