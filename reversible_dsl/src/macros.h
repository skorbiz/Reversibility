/*
 * macros.h
 *
 *  Created on: May 16, 2017
 *      Author: josl
 */

#ifndef REVERSIBLE_DSL_SRC_MACROS_H_
#define REVERSIBLE_DSL_SRC_MACROS_H_


#define MACRO_VARIABLE_TO_STRING(Variable) (void(Variable),#Variable)
#define MACRO_FUNCTION_TO_STRING(Function) (void(&Function),#Function)
#define MACRO_METHOD_TO_STRING(ClassName,Method) (void(&ClassName::Method),#Method)
#define MACRO_TYPE_TO_STRING(Type) (void(sizeof(Type)),#Type)


#endif /* REVERSIBLE_DSL_SRC_MACROS_H_ */









/*
 * int GetAndPrintValue(const char* VariableName)
{
   std::cout << VariableName << std::endl;
   return 10;
}

int Variable=GetAndPrintValue(MACRO_VARIABLE_TO_STRING(Variable));
 */
