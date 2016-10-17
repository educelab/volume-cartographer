//
// Created by Hannah Hatch on 10/17/16.
//

#pragma once

/**
 * @class  TestClass
 * @brief A class used as a template for Doxygen
 *
 * First use class command to specify class name, the use brief for a brief description
 *
 * This is where you will write a longer description of your class
 *
 * @ingroup Example
 * Use this command to specifiy which library your code belongs to, for example Meshing or Texturing
 *
 * Then use see command to specify any other libraries referenced in your code or any other references that can
 * help the user understand how this works
 *
 * @see examples/LoremIpsumError.cpp
 *
 * @warning There is no way to create a member of this class without having at least one parameter
 */
class TestClass{
public:
    /**
     * @brief a brief description of the function
     *
     * A longer description of the function
     *
     * @param parameter1 brief description of this and how it's used
     *
     * See below for an example
     */
    TestClass(int parameter);

    /**
     * @brief Initializes an empty TestClass object and sets both parameters
     *
     * This function takes in two integers and then saves them to that particular objects
     * so that they can be used later
     *
     * @param parameter1 The integer that represents the first parameter
     * @param parameter2 The integer that represents the second parameter
     * @return
     */
    TestClass(int parameter1, int parameter2);

    /** @name Group1 */
     //@{
    /**
     * @brief Gets the value for Parameter1
     * @return The first parameter
     */
    int getParam1() const {return private_param1;}
    /**
     * @brief Gets the value for Parameter2
     * @return The second Parameter
     */
    int getParam2() const {return private_param2; }
    //@}

    /**
     * @brief Adds the two parameters and returns the value
     *
     * Takes the most recent values for parameter1 and parameter2 and then adds them together
     * and returns the value
     * @return the added value of the parameters
     */
    int addParams() const { return (private_param1 + private_param2); }

    /**
     * @brief Compares the parameters  and returns the result
     * @return 'TRUE' if parameter1 is bigger than parameter2
     */
    bool OneBiggerThanTwo() const {return private_param1 > private_param2; };

private:
    /** The first parameter */
    int private_param1;

    /** The second parameter */
    int private_param2;

};


