/*!
 This class is an example class for the commenting and coding style.
 The coding style conforms to the Qt coding standard.
 The commenting style leverages doxygen auto documentation.

 Each revision should be authored and dated with comments.

 First revision, this is the initial class.
 @author jdavis45
 @date 02/21/2013

 Second revision, this is the second revision of the class. Added function.
 @author darwin
 @date 02/30/2013

 */
#ifndef EXAMPLECODECLASS_H
#define EXAMPLECODECLASS_H

#include <string>

class ExampleCodeClass
{
public:
    ExampleCodeClass();

    /*!
     This method will print a string to standard output.
     It should be noted if using QtCreator the comments
     will automatically be created with /\*! <return> on the
     line before the declaration.

      \brief PrintThis
      \param str string to be printed
     */
    void PrintThis(const std::string& str);
};

#endif // EXAMPLECODECLASS_H
