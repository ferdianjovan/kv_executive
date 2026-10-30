// Run the existing Python node with the installed, same-version shared runtime.
// This is the installed node runtime; it does not restart a failed node.
#include <Python.h>

#include <cstdio>
#include <cstring>
#include <vector>

#ifndef KV_PYTHON_EXECUTABLE
#error "KV_PYTHON_EXECUTABLE must identify the configured Python interpreter"
#endif

namespace
{
int initialization_failure(PyStatus status, PyConfig * config)
{
  PyConfig_Clear(config);
  if (PyStatus_IsExit(status)) {
    return status.exitcode;
  }
  Py_ExitStatusException(status);
  return 1;
}
}  // namespace

int main(int argc, char ** argv)
{
  const char * interpreter = KV_PYTHON_EXECUTABLE;
  int first_argument = 1;
  if (argc > 1 && std::strcmp(argv[1], "--python") == 0) {
    if (argc < 3 || argv[2][0] != '/') {
      std::fprintf(stderr, "--python requires an absolute interpreter path\n");
      return 2;
    }
    interpreter = argv[2];
    first_argument = 3;
  }
  if (first_argument < argc && std::strcmp(argv[first_argument], "--") == 0) {
    ++first_argument;
  }

  // Preserve normal Python argument parsing, signals, environment, venv and
  // sys.executable. Only the executable's linked Python runtime is different.
  std::vector<char *> python_arguments;
  python_arguments.push_back(const_cast<char *>(interpreter));
  for (int index = first_argument; index < argc; ++index) {
    python_arguments.push_back(argv[index]);
  }
  PyConfig config;
  PyConfig_InitPythonConfig(&config);
  config.faulthandler = 1;
  PyStatus status = PyConfig_SetBytesString(&config, &config.program_name, interpreter);
  if (PyStatus_Exception(status)) {
    return initialization_failure(status, &config);
  }
  status = PyConfig_SetBytesString(&config, &config.executable, interpreter);
  if (PyStatus_Exception(status)) {
    return initialization_failure(status, &config);
  }
  status = PyConfig_SetBytesArgv(
    &config, static_cast<int>(python_arguments.size()), python_arguments.data());
  if (PyStatus_Exception(status)) {
    return initialization_failure(status, &config);
  }
  status = Py_InitializeFromConfig(&config);
  if (PyStatus_Exception(status)) {
    return initialization_failure(status, &config);
  }
  PyConfig_Clear(&config);
  std::fprintf(stderr, "[kv_shared_python] Using installed Python %.6s shared runtime (%s)\n",
    Py_GetVersion(), interpreter);
  return Py_RunMain();
}
