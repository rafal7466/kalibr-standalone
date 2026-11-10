# Copies every non-system DLL an executable depends on (recursively) next to it,
# so the program runs from any shell regardless of which MSYS2/MinGW is on PATH.
#
# Usage (script mode):
#   cmake -DEXE=<path/to/app.exe> -DSEARCH_DIRS=<dir1;dir2> -DOBJDUMP=<objdump> -P CopyRuntimeDlls.cmake
cmake_minimum_required(VERSION 3.16)
if(POLICY CMP0207)
  cmake_policy(SET CMP0207 NEW)
endif()
file(TO_CMAKE_PATH "${SEARCH_DIRS}" SEARCH_DIRS)
file(TO_CMAKE_PATH "${EXE}" EXE)

if(NOT EXE OR NOT EXISTS "${EXE}")
  message(FATAL_ERROR "CopyRuntimeDlls: EXE='${EXE}' does not exist")
endif()

set(CMAKE_GET_RUNTIME_DEPENDENCIES_PLATFORM "windows+pe")
set(CMAKE_GET_RUNTIME_DEPENDENCIES_TOOL "objdump")
set(CMAKE_GET_RUNTIME_DEPENDENCIES_COMMAND "${OBJDUMP}")

file(GET_RUNTIME_DEPENDENCIES
  EXECUTABLES "${EXE}"
  RESOLVED_DEPENDENCIES_VAR deps
  UNRESOLVED_DEPENDENCIES_VAR unresolved
  DIRECTORIES ${SEARCH_DIRS}
  PRE_EXCLUDE_REGEXES "^[Aa][Pp][Ii]-[Mm][Ss]-.*" "^[Ee][Xx][Tt]-[Mm][Ss]-.*"
  POST_EXCLUDE_REGEXES ".*[/\\\\][Ww][Ii][Nn][Dd][Oo][Ww][Ss][/\\\\].*")

get_filename_component(dst "${EXE}" DIRECTORY)
set(n 0)
foreach(d IN LISTS deps)
  file(COPY "${d}" DESTINATION "${dst}")
  math(EXPR n "${n} + 1")
endforeach()

# Unresolved entries are Windows system DLLs (KERNEL32.dll, ...) that live
# outside SEARCH_DIRS; anything else is worth a warning.
foreach(u IN LISTS unresolved)
  string(TOLOWER "${u}" ul)
  if(NOT ul MATCHES "^(kernel32|user32|advapi32|gdi32|shell32|ole32|oleaut32|ws2_32|comdlg32|comctl32|bcrypt|crypt32|secur32|shlwapi|winmm|version|dbghelp|setupapi|cfgmgr32|imm32|uxtheme|dwmapi|rpcrt4|ntdll|msvcrt|ucrtbase|opengl32|glu32|mfplat|mf|mfreadwrite|mfuuid|d3d11|dxgi|propsys|shcore|iphlpapi|userenv|wsock32|netapi32|ncrypt|psapi|winspool\\.drv|vcruntime140|msvcp140)(\\.dll)?$")
    message(WARNING "CopyRuntimeDlls: could not resolve ${u}")
  endif()
endforeach()
message(STATUS "CopyRuntimeDlls: ${n} DLLs copied next to ${EXE}")
