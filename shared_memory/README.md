Description of package "shared_memory":

1. This package is used to generate a shared library "libshared_memory.so".
    To used the library, the path of the library and header files should be
    provided.

2. "test/test_rw_shm.cpp" is used to test the shared memory operation.
    Usage:
    ros2 run shared_memory test_rw_shm

3. "test/test_sem.cpp" is used to test the semaphore operation.
    Usage:
    ros2 run shared_memory test_sem

4. "test/test_shm_with_sem_write.cpp" is used to test the shared memory operation
    combaining with semaphore.
    Usage:
    ros2 run shared_memory test_shm_with_sem_write

5. "test/test_shm_with_sem_read.cpp" is used to test the shared memory operation
    combaining with semaphore.
    Usage:
    ros2 run shared_memory test_shm_with_sem_read
