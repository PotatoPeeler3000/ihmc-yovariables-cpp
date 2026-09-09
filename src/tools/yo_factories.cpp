#include "ihmc/yovariables/tools/yo_factories.h"

#include "ihmc/yovariables/registry/yo_namespace.h"
#include "ihmc/yovariables/registry/yo_registry.h"

namespace ihmc::yovariables::tools
{
std::vector<std::unique_ptr<registry::YoRegistry>> createChainOfRegistries(const registry::YoNamespace& fullNamespace)
{
   std::vector<std::unique_ptr<registry::YoRegistry>> chain;
   chain.push_back(std::make_unique<registry::YoRegistry>(fullNamespace.getRootName()));
   registry::YoRegistry* current = chain.back().get();

   for (std::size_t i = 1; i < fullNamespace.size(); i++)
   {
      chain.push_back(std::make_unique<registry::YoRegistry>(fullNamespace.getSubName(i)));
      registry::YoRegistry* child = chain.back().get();
      current->addChild(child);
      current = child;
   }

   return chain;
}

registry::YoRegistry* findOrCreateRegistry(registry::YoRegistry& startRegistry, const registry::YoNamespace& fullNamespace,
                                            std::vector<std::unique_ptr<registry::YoRegistry>>& newRegistriesOwnershipSink)
{
   const registry::YoNamespace& namespaceValue = startRegistry.getNamespace();

   if (namespaceValue == fullNamespace)
      return &startRegistry;

   if (!fullNamespace.startsWith(namespaceValue))
      return nullptr;

   for (registry::YoRegistry* child : startRegistry.getChildRegistries())
   {
      registry::YoRegistry* found = findOrCreateRegistry(*child, fullNamespace, newRegistriesOwnershipSink);
      if (found != nullptr)
         return found;
   }

   // None of the children matched: create the rest of the chain here and attach it.
   registry::YoNamespace namespaceToContinueWith = *fullNamespace.removeStart(namespaceValue);
   std::vector<std::unique_ptr<registry::YoRegistry>> chain = createChainOfRegistries(namespaceToContinueWith);

   registry::YoRegistry* chainRoot = chain.front().get();
   registry::YoRegistry* chainBottom = chain.back().get();
   startRegistry.addChild(chainRoot);

   for (std::unique_ptr<registry::YoRegistry>& registryToTransfer : chain)
      newRegistriesOwnershipSink.push_back(std::move(registryToTransfer));

   return chainBottom;
}
}
