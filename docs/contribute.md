# Contribute

We will be happy, if you like to extend or update the error patterns. They are still just a small 
amount of all reasonable error patterns and therefore, you are free to extend them. 

## Workflow

1. Fork this git project
2. Implement awesome stuff
    - For implementing a new feature: 
        1. Create a new abstract method in `include/roboy_error_detection/roboyErrorDetection.hpp`
        2. Write the implementation of this method in `src/roboyErrorDetection.cpp`
        3. Add the new notification codes and descriptions in `include/roboy_error_detection/common_utilities.hpp`
        4. Write an example main file in `src/main` to show how your new functionality works
        5. Add the main file to the `CMakeLists.txt` file to make this file visible for the compiler
    - Change existing code
3. **Ensure that your code compiles**
4. Create a pull request to this git repository