/**
 * @file GenericHeaderExample.h
 * @author Hannah Hatch
 * @date 10/18/16
 *
 * @brief Brief description of a generic, non-class oriented header
 *
 * This is the long description to explain what this header has in it. There
 * should be a single blank line between the brief description and this. It can
 * be multiple lines long. In fact...
 *
 * @newline
 * ...you can also use the newline command to create a new line to separate
 * large blocks of text.
 *
 * @newline
 * Also, don't forget to add this header to a Module using the `ingroup`
 * command. All of the objects in this file will be added to the specified
 * group.
 *
 * @ingroup ExampleDocs
 */

#pragma once

#include <string>

/**
 * typedefs should be declared with the `using` syntax
 */
using ExampleString = std::string;

/**
 * @brief Brief descriptions can be used...
 *
 * ...and long descriptions work like they do with classes.
 */
const ExampleString GlobalString1 = "Test String";

/** Descriptions without the `brief` command will be interpretted as long
 * descriptions.
 */
const ExampleString GlobalString2 = "Test String";
