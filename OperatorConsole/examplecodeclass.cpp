#include "examplecodeclass.h"

ExampleCodeClass::ExampleCodeClass()
{
}

void ExampleCodeClass::PrintThis(const std::string &str)
{
    printf(str.c_str());
}
