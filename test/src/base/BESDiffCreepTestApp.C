//* This file is part of the MOOSE framework
//* https://www.mooseframework.org
//*
//* All rights reserved, see COPYRIGHT for full restrictions
//* https://github.com/idaholab/moose/blob/master/COPYRIGHT
//*
//* Licensed under LGPL 2.1, please see LICENSE for details
//* https://www.gnu.org/licenses/lgpl-2.1.html
#include "BESDiffCreepTestApp.h"
#include "BESDiffCreepApp.h"
#include "Moose.h"
#include "AppFactory.h"
#include "MooseSyntax.h"
#include "ModulesApp.h"

InputParameters
BESDiffCreepTestApp::validParams()
{
  InputParameters params = BESDiffCreepApp::validParams();
  return params;
}

BESDiffCreepTestApp::BESDiffCreepTestApp(InputParameters parameters) : MooseApp(parameters)
{
  BESDiffCreepTestApp::registerAll(
      _factory, _action_factory, _syntax, getParam<bool>("allow_test_objects"));
}

BESDiffCreepTestApp::~BESDiffCreepTestApp() {}

void
BESDiffCreepTestApp::registerAll(Factory & f, ActionFactory & af, Syntax & s, bool use_test_objs)
{
  BESDiffCreepApp::registerAll(f, af, s);
  if (use_test_objs)
  {
    Registry::registerObjectsTo(f, {"BESDiffCreepTestApp"});
    Registry::registerActionsTo(af, {"BESDiffCreepTestApp"});
  }
}

void
BESDiffCreepTestApp::registerApps()
{
  registerApp(BESDiffCreepApp);
  registerApp(BESDiffCreepTestApp);
}

/***************************************************************************************************
 *********************** Dynamic Library Entry Points - DO NOT MODIFY ******************************
 **************************************************************************************************/
// External entry point for dynamic application loading
extern "C" void
BESDiffCreepTestApp__registerAll(Factory & f, ActionFactory & af, Syntax & s)
{
  BESDiffCreepTestApp::registerAll(f, af, s);
}
extern "C" void
BESDiffCreepTestApp__registerApps()
{
  BESDiffCreepTestApp::registerApps();
}
