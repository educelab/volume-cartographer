#pragma once

/**
 * @class  ClassName
 * @brief A brief description of the glass
 *
 * @author Hannah Hatch
 * @date 10/17/16
 *
 * Long description for the class, separated by a single, blank line.
 * It can be multiple lines long
 *
 * @ingroup ExampleDocs
 *
 * @see Other relevant files
 *
 *
 */
class TestClass
{
public:
    /**
     * @brief The initializer
     *
     * This creates an object of TestClass and stores parameter into
     * private_param1
     *
     * @param parameter The integer that is saved
     *
     */
    TestClass(int parameter);

    /**
     * @brief Initializes an empty TestClass object and sets both parameters
     *
     * This function takes in two integers and then saves them to that
     * particular objects so that they can be used later
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
    int getParam1() const { return private_param1; }
    /**
     * @brief Gets the value for Parameter2
     * @return The second Parameter
     */
    int getParam2() const { return private_param2; }
    //@}

    /**
     * @brief Adds the two parameters and returns the value
     *
     * Takes the most recent values for parameter1 and parameter2 and then adds
     * them together and returns the value
     *
     * @warning This function can do something you don't expect
     * @return the added value of the parameters
     */
    int addParams() const { return (private_param1 + private_param2); }

    /**
     * @brief Compares the parameters  and returns the result
     *
     * \f[ param1 > param2 \f]
     *
     * @return 'TRUE' if parameter1 is bigger than parameter2
     */
    bool OneBiggerThanTwo() const { return private_param1 > private_param2; };

    /**
     * The description written in this comment block for the const function will
     * also show up for the non const version.
     */
    int swapParams() const;

    /** @copydoc TestClass::swapParams() const */
    int swapParams();

private:
    /** The first parameter */
    int private_param1;

    /** The second parameter */
    int private_param2;
};
