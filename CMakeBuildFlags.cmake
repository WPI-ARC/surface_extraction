set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O1 -fsanitize=address -fno-omit-frame-pointer -std=c++11 -Wall -Wextra -Wno-unused-parameter -Wno-unused-local-typedefs -Wno-sign-conversion")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O1 -fsanitize=memory -fno-omit-frame-pointer -std=c++11 -Wall -Wextra -Wno-unused-parameter -Wno-unused-local-typedefs")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -std=c++11 -Wall -Wextra -Wno-unused-parameter -Wno-unused-local-typedefs")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O2 -std=c++11 -Wall -Wextra -Wno-unused-parameter -Wno-unused-local-typedefs")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DNDEBUG -O3 -std=c++11 -Wall -Wextra -Wno-unused-parameter -Wno-unused-local-typedefs")

# PCL 1.8 has some issues that may be fixed by this
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -msse4.2 -mfpmath=sse")