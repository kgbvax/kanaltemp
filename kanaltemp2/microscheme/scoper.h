/*	microScheme Scoper Header File	*/

#ifndef _SCOPER_H_GUARD
#define _SCOPER_H_GUARD

// Type definitions
	typedef struct Environment { char **binding; int *lexicalAddrV; int *lexicalAddrH; int *references; int *realAddress; int numBinds; struct Environment *enclosing; } Environment;

// Shared variable declarations
	extern Environment *currentEnvironment;
	extern Environment *currentClosureEnvironment;

// Shared function declarations
	extern AST_expr *scoper_scopeExpr(AST_expr *expr);
	extern void freeEnvironment(Environment *env);
	extern void scoper_initEnv(Environment *env);


#endif

