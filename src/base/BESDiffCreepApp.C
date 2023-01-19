#include "BESDiffCreepApp.h"
#include "Moose.h"
#include "AppFactory.h"
#include "ModulesApp.h"
#include "MooseSyntax.h"

InputParameters
BESDiffCreepApp::validParams()
{
  InputParameters params = MooseApp::validParams();

  return params;
}

BESDiffCreepApp::BESDiffCreepApp(InputParameters parameters) : MooseApp(parameters)
{
  BESDiffCreepApp::registerAll(_factory, _action_factory, _syntax);
}

BESDiffCreepApp::~BESDiffCreepApp() {}

void
BESDiffCreepApp::registerAll(Factory & f, ActionFactory & af, Syntax & syntax)
{
  ModulesApp::registerAll(f, af, syntax);
  Registry::registerObjectsTo(f, {"BESDiffCreepApp"});
  Registry::registerActionsTo(af, {"BESDiffCreepApp"});

  /* register custom execute flags, action syntax, etc. here */
}

void
BESDiffCreepApp::registerApps()
{
  registerApp(BESDiffCreepApp);
}

/***************************************************************************************************
 *********************** Dynamic Library Entry Points - DO NOT MODIFY ******************************
 **************************************************************************************************/
extern "C" void
BESDiffCreepApp__registerAll(Factory & f, ActionFactory & af, Syntax & s)
{
  BESDiffCreepApp::registerAll(f, af, s);
}
extern "C" void
BESDiffCreepApp__registerApps()
{
  BESDiffCreepApp::registerApps();
}
