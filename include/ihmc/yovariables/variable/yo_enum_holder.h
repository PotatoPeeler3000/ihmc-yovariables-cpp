#pragma once

#include <string>
#include <vector>

namespace ihmc::yovariables::variable
{
/**
 * Type-erased, non-template access to the parts of YoEnum&lt;E&gt; that do not depend on the enum
 * type E. Unlike Java, where a raw YoEnum&lt;?&gt; reference can still be introspected generically
 * at runtime, C++ template instantiations of YoEnum&lt;E&gt; for different E share no common base
 * beyond YoVariable - so code that walks a YoVariable* tree without knowing E (e.g. the
 * YoVariableServer handshake builder) needs this interface instead.
 */
class YoEnumHolder
{
public:
   virtual ~YoEnumHolder() = default;

   virtual bool isBackedByEnum() const = 0;
   virtual bool isNullAllowed() const = 0;
   virtual int getOrdinal() const = 0;
   virtual const std::vector<std::string>& getEnumValuesAsString() const = 0;

   /**
    * A key stable and unique per enum type E within this process, standing in for Java's
    * `Class&lt;E&gt;.getCanonicalName()` (which has no C++ equivalent - RTTI type names are not
    * portable/stable strings the way Java class names are). Used only to deduplicate identical enum
    * types across multiple YoEnum variables when building a YoVariableServer handshake; unlike Java,
    * this key has no meaning outside this process and is never sent to the wire.
    */
   virtual std::string getEnumTypeKey() const = 0;
};
}
