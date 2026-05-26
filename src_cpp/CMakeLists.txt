# 1. Create a variable called SOURCES that holds a list of your .cpp files.
# Whenever you add a new controller, you just add its filename to this list.
set(SOURCES
    main.cpp
    plant/ManipulatorPlant.cpp
    trajectory/TrajectoryGenerator.cpp
    models/RobotModel.cpp
    ipc/SharedMemoryLink.cpp
)

# 2. Create the actual executable program and name it "RoboticSim".
# We pass it the ${SOURCES} variable so it knows which files to compile.
add_executable(RoboticSim ${SOURCES})

# 3. Tell the compiler where your header files (.h) live.
# "PRIVATE" means these headers are only used by RoboticSim, not shared with other targets.
target_include_directories(RoboticSim PRIVATE include plant trajectory controllers models ipc sensors graphics)
# Tell RoboticSim it is allowed to read files from the vendored eigen folder
target_include_directories(RoboticSim PRIVATE ${CMAKE_SOURCE_DIR}/vendor/eigen)

# 4. Link the necessary libraries to your executable.
target_link_libraries(RoboticSim
    PRIVATE
    Boost::boost   # Links the Boost headers for Boost.Interprocess
    pthread        # POSIX Threading: Required by Boost under the hood
    rt             # POSIX Real-Time: Absolutely required for Linux shared memory (shm_open)
)

# 5. Create the separate graphics application executable
add_executable(GraphicsApp 
    graphics/GraphicsApp.cpp 
    graphics/GraphicsEngine.cpp 
    ipc/SharedMemoryLink.cpp
    ${CMAKE_SOURCE_DIR}/vendor/glad/src/glad.c
)
target_include_directories(GraphicsApp PRIVATE 
    include 
    ipc 
    graphics
    ${CMAKE_SOURCE_DIR}/vendor/glad/include
    ${CMAKE_SOURCE_DIR}/vendor/glm
)
target_link_libraries(GraphicsApp PRIVATE pthread rt glfw GL dl)
