#include "ihmc/yovariables/parameters/boolean_parameter.h"

namespace ihmc::yovariables::parameters
{
BooleanParameter::BooleanParameter(const std::string& name, registry::YoRegistry* registry) : BooleanParameter(name, "", registry, false)
{
}

BooleanParameter::BooleanParameter(const std::string& name, const std::string& description, registry::YoRegistry* registry)
   : BooleanParameter(name, description, registry, false)
{
}

BooleanParameter::BooleanParameter(const std::string& name, registry::YoRegistry* registry, bool initialValue)
   : BooleanParameter(name, "", registry, initialValue)
{
}

BooleanParameter::BooleanParameter(const std::string& name, const std::string& description, registry::YoRegistry* registry, bool initialValue)
   : value_(std::make_unique<BackingVariable>(*this, name, description, registry)), initialValue_(initialValue)
{
   setParameterBounds(0, 1);
}

bool BooleanParameter::getValue() const
{
   checkLoaded();
   return value_->getBooleanValue();
}

variable::YoBoolean& BooleanParameter::getVariable() const
{
   return *value_;
}

void BooleanParameter::setToDefault()
{
   value_->set(initialValue_);
}

BooleanParameter::BackingVariable::BackingVariable(BooleanParameter& owner, const std::string& name, const std::string& description,
                                                     registry::YoRegistry* registry)
   : variable::YoBoolean(name, description, nullptr), owner_(owner)
{
   // Registering with `registry` is deferred to here (the constructor body) rather than passed to
   // the YoBoolean base constructor above: while a base class subobject is under construction, C++
   // dispatches virtual calls using that base's own vtable, not the eventually-most-derived one (Java
   // does not have a base/derived-vtable distinction during construction, so the equivalent Java code
   // has no such issue). Registering earlier would mean YoRegistry::addVariable's calls to
   // isParameter()/getParameter() resolve to YoVariable's un-overridden defaults instead of this
   // class's overrides, silently leaving this parameter out of the registry's parameters list.
   setRegistry(registry);
}

std::unique_ptr<variable::YoVariable> BooleanParameter::BackingVariable::duplicate(registry::YoRegistry* newRegistry) const
{
   // Deliberately leaked: getParameter() on the returned variable points back at this shell
   // BooleanParameter, so it must stay alive for exactly as long as the returned variable does - a
   // lifetime intertwined with what we're returning, which unique_ptr<YoVariable>'s single-owner
   // contract (fixed by the base class interface) cannot express directly. Java's version has the
   // same shape of relationship (the returned YoVariable and the new parameter reference each other)
   // and relies on the GC to collect the pair together once both become unreachable; duplicate() is
   // not on any hot/frequently-called path, so trading that GC behavior for a bounded one-shell leak
   // per call is preferable here to unsafe shared ownership or a wider interface change.
   BooleanParameter* newParameter = new BooleanParameter(getName(), getDescription(), newRegistry, owner_.initialValue_);
   newParameter->value_->set(getValue());
   newParameter->setLoadStatus(owner_.getLoadStatus());
   // .get(), not .release(): newParameter->value_ deliberately stays populated (rather than null)
   // so the leaked shell above remains a fully functional BooleanParameter, e.g. if something reaches
   // it via getParameter() on the returned variable. This does not double-free: newParameter is never
   // deleted, so its value_ member's own destructor never runs - the unique_ptr<YoVariable> returned
   // here is the only one that will ever actually delete the BackingVariable both are pointing at.
   return std::unique_ptr<variable::YoVariable>(newParameter->value_.get());
}
}
