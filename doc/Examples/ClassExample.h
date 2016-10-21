#pragma once

/**
 * @class ClassExample
 * @author Hannah Hatch
 * @date 10/17/16
 *
 * @brief A brief description of a class
 *
 * Long description for the class, separated by a single, blank line.
 * It can be multiple lines long, but the text will flow together in a single
 * paragraph.
 *
 * @newline
 * You can use the `newline` command to separate paragraphs.
 *
 * @newline
 * A note on style: Brief descriptions do not need to end in a
 * period, but long descriptions should.
 *
 * @ingroup ExampleDocs
 *
 * @see doc/Examples/ClassExample.cpp
 */
class ClassExample
{
public:
    /**
     * @brief a brief description of the function
     *
     * A longer description of the function.
     *
     * @param parameter Brief description of the parameter
     *
     */
    ClassExample(int parameter);

    /**
     * @brief Initializes an empty ClassExample object and sets both parameters
     *
     * This function takes in two integers and then saves them to that
     * particular objects so that they can be used later.
     *
     * @param parameter1 The integer that represents the first parameter
     * @param parameter2 The integer that represents the second parameter
     */
    ClassExample(int parameter1, int parameter2);

    /** @name Group1 */
    //@{
    /**
     * @brief Gets the value for Parameter1
     * @return The first parameter
     */
    int getParam1() const { return private_param1; }
    /**
     * @brief Gets the value for Parameter2
     * @return The second parameter
     */
    int getParam2() const { return private_param2; }
    //@}

    /**
     * @brief Adds the two parameters and returns the value
     *
     * Takes the most recent values for parameter1 and parameter2 and then adds
     * them together and returns the value
     *
     * @warning This function can do something you don't expect. Use the
     * warning function to notify the developer of this.
     * @return The sum of the parameters
     */
    int addParams() const { return (private_param1 + private_param2); }

    /**
     * @brief Returns greater-than comparison for param1 & param2
     *
     * Compares parameters and returns the result of:
     * \f[ param1 > param2 \f]
     * Note that this demonstrates how LaTeX mathematics can be used in
     * descriptions.
     *
     * @return `TRUE` if parameter1 is bigger than parameter2
     */
    bool OneBiggerThanTwo() const { return private_param1 > private_param2; };

    /**
     * The description written in this comment block for the const function will
     * also show up for the non const version because we used the copydoc
     * command.
     */
    int swapParams() const;

    /** @copydoc ClassExample::swapParams() const */
    int swapParams();

private:
    /** The first parameter */
    int private_param1;

    /** The second parameter */
    int private_param2;
};
