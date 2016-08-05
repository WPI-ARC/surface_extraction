set(CMAKE_BUILD_TYPE Debug)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O1 -fsanitize=address -fno-omit-frame-pointer -std=c++11 -Wall -Wextra -Wno-unused-parameter -Wno-unused-local-typedefs")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -std=c++11 -Wall -Wextra -Wno-unused-parameter -Wno-unused-local-typedefs")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O2 -std=c++11 -Wall -Wextra -Wno-unused-parameter -Wno-unused-local-typedefs")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DNDEBUG -O3 -std=c++11 -Wall -Wextra -Wno-unused-parameter -Wno-unused-local-typedefs")