#include "StorageRuntime.h"

#ifdef STORAGE_PICK_MODE
#define PICK_STORAGE_TEST
#endif
#define setup StorageTestRuntime_Setup
#define loop StorageTestRuntime_Loop
#include "StorageRuntimeImpl.h"
#undef setup
#undef loop
