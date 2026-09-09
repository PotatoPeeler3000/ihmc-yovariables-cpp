#include "ihmc/yovariables/buffer/yo_buffer.h"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

#include "ihmc/yovariables/registry/yo_namespace.h"
#include "ihmc/yovariables/tools/yo_tools.h"
#include "ihmc/yovariables/variable/yo_variable.h"

namespace ihmc::yovariables::buffer
{
namespace
{
std::string toLower(const std::string& value)
{
   std::string result = value;
   std::transform(result.begin(), result.end(), result.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
   return result;
}

bool epsilonEquals(double a, double b, double epsilon)
{
   return std::abs(a - b) <= epsilon;
}
} // namespace

YoBuffer::YoBuffer(int bufferSize) : bufferSize_(bufferSize)
{
}

YoBuffer::YoBuffer(const YoBuffer& other)
   : timeVariableName_(other.timeVariableName_), inPoint_(other.inPoint_), outPoint_(other.outPoint_), currentIndex_(other.currentIndex_),
     bufferSize_(other.bufferSize_), lockIndex_(other.lockIndex_)
{
   for (const std::unique_ptr<YoBufferVariableEntry>& otherEntry : other.entries_)
      addEntry(std::make_unique<YoBufferVariableEntry>(*otherEntry));
}

void YoBuffer::clear()
{
   inPoint_ = 0;
   outPoint_ = 0;
   currentIndex_ = 0;
   entries_.clear();
   simpleNameToEntriesMap_.clear();
   keyPointsHandler_.clear();
   indexChangedListeners_.clear();
}

void YoBuffer::clearBuffers(int bufferSize)
{
   for (std::unique_ptr<YoBufferVariableEntry>& entry : entries_)
      entry->clearBuffer(bufferSize);
   bufferSize_ = bufferSize;
}

void YoBuffer::resizeBuffer(int newBufferSize)
{
   if (newBufferSize < bufferSize_)
   {
      cropBuffer(inPoint_, (inPoint_ + newBufferSize - 1) % bufferSize_);
   }
   else if (newBufferSize > bufferSize_)
   {
      shiftBuffer();
      enlargeBufferSize(newBufferSize);
   }
}

void YoBuffer::enlargeBufferSize(int newBufferSize)
{
   for (std::unique_ptr<YoBufferVariableEntry>& entry : entries_)
      entry->enlargeBufferSize(newBufferSize);

   bufferSize_ = newBufferSize;
}

void YoBuffer::setLockIndex(bool lock)
{
   lockIndex_ = lock;
}

bool YoBuffer::isIndexLocked() const
{
   return lockIndex_;
}

void YoBuffer::addEntry(std::unique_ptr<YoBufferVariableEntry> entry)
{
   if (entry->getBufferSize() != bufferSize_)
      throw std::invalid_argument("The new entry size (" + std::to_string(entry->getBufferSize()) + ") does not match the buffer size ("
                                   + std::to_string(bufferSize_) + ").");

   YoBufferVariableEntry* rawEntry = entry.get();
   entries_.push_back(std::move(entry));

   std::string variableName = toLower(rawEntry->getVariable().getName());
   simpleNameToEntriesMap_[variableName].push_back(rawEntry);
}

YoBufferVariableEntry& YoBuffer::addVariable(variable::YoVariable& variable)
{
   YoBufferVariableEntry* entry = getEntry(variable);
   if (entry != nullptr)
      return *entry;

   auto newEntry = std::make_unique<YoBufferVariableEntry>(variable, bufferSize_);
   YoBufferVariableEntry& result = *newEntry;
   addEntry(std::move(newEntry));
   return result;
}

void YoBuffer::addVariables(const std::vector<variable::YoVariable*>& variables)
{
   entries_.reserve(entries_.size() + variables.size());
   for (variable::YoVariable* variableToAdd : variables)
      addVariable(*variableToAdd);
}

std::unique_ptr<YoBufferVariableEntry> YoBuffer::removeVariable(variable::YoVariable& variable)
{
   YoBufferVariableEntry* entry = getEntry(variable);
   if (entry == nullptr)
      return nullptr;

   auto entryIt = std::find_if(entries_.begin(), entries_.end(), [entry](const std::unique_ptr<YoBufferVariableEntry>& candidate) {
      return candidate.get() == entry;
   });

   std::unique_ptr<YoBufferVariableEntry> removed = std::move(*entryIt);
   entries_.erase(entryIt);

   std::vector<YoBufferVariableEntry*>& sameName = simpleNameToEntriesMap_[toLower(variable.getName())];
   sameName.erase(std::remove(sameName.begin(), sameName.end(), entry), sameName.end());

   return removed;
}

void YoBuffer::setInPoint(int index)
{
   inPoint_ = index;
   keyPointsHandler_.trimKeyPoints(inPoint_, outPoint_);
}

void YoBuffer::setOutPoint(int index)
{
   outPoint_ = index;
   keyPointsHandler_.trimKeyPoints(inPoint_, outPoint_);
}

void YoBuffer::setInPoint()
{
   setInPoint(currentIndex_);
}

void YoBuffer::setOutPoint()
{
   setOutPoint(currentIndex_);
}

void YoBuffer::setInOutPointFullBuffer()
{
   inPoint_ = 0;
   outPoint_ = getBufferSize() - 1;
}

void YoBuffer::toggleKeyPoint()
{
   keyPointsHandler_.toggleKeyPoint(currentIndex_);
}

bool YoBuffer::isAtInPoint() const
{
   return currentIndex_ == inPoint_;
}

bool YoBuffer::isAtOutPoint() const
{
   return currentIndex_ == outPoint_;
}

void YoBuffer::readFromBuffer()
{
   for (std::unique_ptr<YoBufferVariableEntry>& entry : entries_)
      entry->readFromBufferAt(currentIndex_);
}

void YoBuffer::writeIntoBuffer()
{
   for (std::unique_ptr<YoBufferVariableEntry>& entry : entries_)
      entry->writeIntoBufferAt(currentIndex_);
}

void YoBuffer::gotoInPoint()
{
   setCurrentIndex(inPoint_);
}

void YoBuffer::gotoOutPoint()
{
   setCurrentIndex(outPoint_);
}

void YoBuffer::setCurrentIndex(int index)
{
   if (lockIndex_)
      return;

   currentIndex_ = index;

   if (currentIndex_ >= bufferSize_)
      currentIndex_ = 0;
   else if (currentIndex_ < 0)
      currentIndex_ = bufferSize_ - 1;

   readFromBuffer();
   notifyIndexChangedListeners();
}

bool YoBuffer::tickAndReadFromBuffer(int stepSize)
{
   if (lockIndex_)
      return false;

   int newIndex = currentIndex_ + stepSize;
   bool rolledOver = !isIndexBetweenBounds(newIndex);

   if (rolledOver)
      newIndex = stepSize >= 0 ? inPoint_ : outPoint_;

   setCurrentIndex(newIndex);

   return rolledOver;
}

void YoBuffer::tickAndWriteIntoBuffer()
{
   // Deliberately ignores lockIndex_ - inconsistent with the rest of the API, but needed for SCS
   // to function properly when used as a remote visualizer (preserved from the Java source).
   currentIndex_ = currentIndex_ + 1;

   if (currentIndex_ >= bufferSize_ || currentIndex_ < 0)
      currentIndex_ = 0;

   outPoint_ = currentIndex_;

   if (outPoint_ == inPoint_)
   {
      inPoint_++;
      if (inPoint_ >= bufferSize_)
         inPoint_ = 0;
   }

   keyPointsHandler_.removeKeyPoint(currentIndex_);
   writeIntoBuffer();
   notifyIndexChangedListeners();
}

void YoBuffer::notifyIndexChangedListeners()
{
   for (interfaces::YoBufferIndexChangedListener* listener : indexChangedListeners_)
      listener->indexChanged(currentIndex_);
}

void YoBuffer::fillBuffer()
{
   for (std::unique_ptr<YoBufferVariableEntry>& entry : entries_)
      entry->fillBuffer();
}

void YoBuffer::shiftBuffer()
{
   shiftBuffer(inPoint_);
}

void YoBuffer::shiftBuffer(int shiftIndex)
{
   if (shiftIndex == 0)
      return;

   if (shiftIndex <= 0 || shiftIndex >= bufferSize_)
      return;

   for (std::unique_ptr<YoBufferVariableEntry>& entry : entries_)
      entry->shiftBuffer(shiftIndex);

   currentIndex_ = (currentIndex_ - shiftIndex + bufferSize_) % bufferSize_;
   if (currentIndex_ < 0)
      currentIndex_ = 0;

   inPoint_ = 0;
   outPoint_ = (outPoint_ - shiftIndex + bufferSize_) % bufferSize_;

   tickAndReadFromBuffer(0);
}

void YoBuffer::cropBuffer()
{
   if (inPoint_ != outPoint_)
      cropBuffer(inPoint_, outPoint_);
   else
      cropBuffer(inPoint_, inPoint_ + 1);
}

void YoBuffer::cropBuffer(int start, int end)
{
   if (start < 0 || end > bufferSize_)
      return;

   bufferSize_ = YoBufferVariableEntry::computeBufferSizeAfterCrop(start, end, bufferSize_);

   for (std::unique_ptr<YoBufferVariableEntry>& entry : entries_)
      entry->cropBuffer(start, end);

   currentIndex_ = (currentIndex_ - start + bufferSize_) % bufferSize_;
   if (currentIndex_ < 0 || currentIndex_ >= bufferSize_)
      currentIndex_ = 0;

   inPoint_ = 0;
   outPoint_ = bufferSize_ - 1;

   gotoInPoint();
}

void YoBuffer::cutBuffer()
{
   if (inPoint_ <= outPoint_)
      cutBuffer(inPoint_, outPoint_);
}

void YoBuffer::cutBuffer(int start, int end)
{
   if (start < 0 || end > bufferSize_ || start > end)
      return;

   bufferSize_ = YoBufferVariableEntry::computeBufferSizeAfterCut(start, end, bufferSize_);

   for (std::unique_ptr<YoBufferVariableEntry>& entry : entries_)
   {
      int actualBufferSize = entry->cutBuffer(start, end);
      if (actualBufferSize >= 0)
         bufferSize_ = actualBufferSize;
   }

   currentIndex_ = (currentIndex_ - start + bufferSize_) % bufferSize_;
   if (currentIndex_ < 0 || currentIndex_ >= bufferSize_)
      currentIndex_ = 0;

   inPoint_ = 0;
   outPoint_ = start - 1;

   gotoOutPoint();
}

void YoBuffer::thinData(int n)
{
   shiftBuffer();

   inPoint_ = 0;
   currentIndex_ = 0;

   if (bufferSize_ <= 2 * n)
      return;

   for (std::unique_ptr<YoBufferVariableEntry>& entry : entries_)
   {
      int newBufferSize = entry->thinData(n);
      if (newBufferSize >= 0)
         bufferSize_ = newBufferSize;
   }

   outPoint_ = bufferSize_ - 1;

   gotoInPoint();
}

double YoBuffer::computeAverage(variable::YoVariable& variable) const
{
   YoBufferVariableEntry* entry = const_cast<YoBuffer*>(this)->getEntry(variable);
   if (entry == nullptr)
      return std::nan("");
   return entry->computeAverage();
}

void YoBuffer::applyProcessor(interfaces::YoBufferProcessor& processor)
{
   processor.initialize(*this);

   if (processor.goForward())
   {
      gotoInPoint();

      while (!isAtOutPoint())
      {
         processor.process(inPoint_, outPoint_, currentIndex_);
         writeIntoBuffer();
         tickAndReadFromBuffer(1);
      }

      processor.process(inPoint_, outPoint_, currentIndex_);
      writeIntoBuffer();
      tickAndReadFromBuffer(1);
   }
   else
   {
      gotoOutPoint();

      while (!isAtInPoint())
      {
         processor.process(outPoint_, inPoint_, currentIndex_);
         writeIntoBuffer();
         tickAndReadFromBuffer(-1);
      }

      processor.process(outPoint_, inPoint_, currentIndex_);
      writeIntoBuffer();
      tickAndReadFromBuffer(-1);
   }
}

int YoBuffer::getInPoint() const
{
   return inPoint_;
}

int YoBuffer::getOutPoint() const
{
   return outPoint_;
}

int YoBuffer::getNextKeyPoint() const
{
   return keyPointsHandler_.getNextKeyPoint(currentIndex_);
}

int YoBuffer::getPreviousKeyPoint() const
{
   return keyPointsHandler_.getPreviousKeyPoint(currentIndex_);
}

int YoBuffer::getBufferSize() const
{
   return bufferSize_;
}

YoBufferVariableEntry* YoBuffer::getEntry(variable::YoVariable& variable)
{
   for (std::unique_ptr<YoBufferVariableEntry>& entry : entries_)
   {
      if (&entry->getVariable() == &variable)
         return entry.get();
   }
   return nullptr;
}

const std::vector<std::unique_ptr<YoBufferVariableEntry>>& YoBuffer::getEntries() const
{
   return entries_;
}

std::vector<variable::YoVariable*> YoBuffer::getVariables()
{
   std::vector<variable::YoVariable*> result;
   result.reserve(entries_.size());
   for (std::unique_ptr<YoBufferVariableEntry>& entry : entries_)
      result.push_back(&entry->getVariable());
   return result;
}

void YoBuffer::addListener(interfaces::YoBufferIndexChangedListener* listenerToAdd)
{
   indexChangedListeners_.push_back(listenerToAdd);
}

void YoBuffer::removeListeners()
{
   indexChangedListeners_.clear();
}

bool YoBuffer::removeListener(interfaces::YoBufferIndexChangedListener* listenerToRemove)
{
   auto it = std::find(indexChangedListeners_.begin(), indexChangedListeners_.end(), listenerToRemove);
   if (it == indexChangedListeners_.end())
      return false;
   indexChangedListeners_.erase(it);
   return true;
}

int YoBuffer::getCurrentIndex() const
{
   return currentIndex_;
}

bool YoBuffer::epsilonEquals(const YoBuffer& other, double epsilon) const
{
   if (entries_.size() != other.entries_.size())
      return false;

   if (getBufferInOutLength() != other.getBufferInOutLength())
      return false;

   int length = getBufferInOutLength();

   for (const std::unique_ptr<YoBufferVariableEntry>& otherEntry : other.entries_)
   {
      variable::YoVariable& otherVariable = otherEntry->getVariable();
      YoBufferVariableEntry* thisEntry = const_cast<YoBuffer*>(this)->findVariableEntry(otherVariable.getFullNameString());

      if (thisEntry == nullptr)
         return false;

      int count = 0;
      int thisIndex = getInPoint();
      int otherIndex = other.getInPoint();

      while (count < length)
      {
         double thisDataPoint = thisEntry->readBufferAt(thisIndex);
         double otherDataPoint = otherEntry->readBufferAt(otherIndex);

         if (thisDataPoint != otherDataPoint && !buffer::epsilonEquals(thisDataPoint, otherDataPoint, epsilon))
            return false;

         count++;
         thisIndex++;
         otherIndex++;

         if (thisIndex >= getBufferSize())
            thisIndex = 0;
         if (otherIndex >= other.getBufferSize())
            otherIndex = 0;
      }
   }

   return true;
}

void YoBuffer::setTimeVariableName(const std::string& timeVariableName)
{
   if (findVariableEntry(timeVariableName) == nullptr)
      return;
   timeVariableName_ = timeVariableName;
}

const std::string& YoBuffer::getTimeVariableName() const
{
   return timeVariableName_;
}

KeyPointsHandler& YoBuffer::getKeyPointsHandler()
{
   return keyPointsHandler_;
}

std::vector<double> YoBuffer::getTimeBuffer() const
{
   return const_cast<YoBuffer*>(this)->findVariableEntry(timeVariableName_)->getBuffer();
}

variable::YoVariable* YoBuffer::findVariable(const std::optional<std::string>& namespaceEnding, const std::string& name)
{
   YoBufferVariableEntry* entry = findVariableEntry(namespaceEnding, name);
   return entry == nullptr ? nullptr : &entry->getVariable();
}

YoBufferVariableEntry* YoBuffer::findVariableEntry(const std::string& name)
{
   std::size_t separatorIndex = name.rfind(tools::kNamespaceSeparator);
   if (separatorIndex == std::string::npos)
      return findVariableEntry(std::nullopt, name);
   return findVariableEntry(name.substr(0, separatorIndex), name.substr(separatorIndex + 1));
}

YoBufferVariableEntry* YoBuffer::findVariableEntry(const std::optional<std::string>& namespaceEnding, const std::string& name)
{
   tools::checkNameDoesNotContainSeparator(name);
   auto it = simpleNameToEntriesMap_.find(toLower(name));
   if (it == simpleNameToEntriesMap_.end() || it->second.empty())
      return nullptr;

   if (!namespaceEnding.has_value())
      return it->second.front();

   for (YoBufferVariableEntry* candidate : it->second)
   {
      const registry::YoNamespace* candidateNamespace = candidate->getVariable().getNamespace();
      if (candidateNamespace != nullptr && candidateNamespace->endsWith(*namespaceEnding, true))
         return candidate;
   }

   return nullptr;
}

std::vector<variable::YoVariable*> YoBuffer::findVariables(const std::optional<std::string>& namespaceEnding, const std::string& name)
{
   std::vector<YoBufferVariableEntry*> matchingEntries = findVariableEntries(namespaceEnding, name);
   std::vector<variable::YoVariable*> result;
   result.reserve(matchingEntries.size());
   for (YoBufferVariableEntry* entry : matchingEntries)
      result.push_back(&entry->getVariable());
   return result;
}

std::vector<YoBufferVariableEntry*> YoBuffer::findVariableEntries(const std::string& name)
{
   std::size_t separatorIndex = name.rfind(tools::kNamespaceSeparator);
   if (separatorIndex == std::string::npos)
      return findVariableEntries(std::nullopt, name);
   return findVariableEntries(name.substr(0, separatorIndex), name.substr(separatorIndex + 1));
}

std::vector<YoBufferVariableEntry*> YoBuffer::findVariableEntries(const std::optional<std::string>& namespaceEnding, const std::string& name)
{
   tools::checkNameDoesNotContainSeparator(name);
   auto it = simpleNameToEntriesMap_.find(toLower(name));
   if (it == simpleNameToEntriesMap_.end() || it->second.empty())
      return {};

   if (!namespaceEnding.has_value())
      return it->second;

   std::vector<YoBufferVariableEntry*> result;
   for (YoBufferVariableEntry* candidate : it->second)
   {
      const registry::YoNamespace* candidateNamespace = candidate->getVariable().getNamespace();
      if (candidateNamespace != nullptr && candidateNamespace->endsWith(*namespaceEnding, true))
         result.push_back(candidate);
   }
   return result;
}

std::vector<variable::YoVariable*> YoBuffer::findVariables(const registry::YoNamespace& namespaceValue)
{
   std::vector<YoBufferVariableEntry*> matchingEntries = findVariableEntries(namespaceValue);
   std::vector<variable::YoVariable*> result;
   result.reserve(matchingEntries.size());
   for (YoBufferVariableEntry* entry : matchingEntries)
      result.push_back(&entry->getVariable());
   return result;
}

std::vector<YoBufferVariableEntry*> YoBuffer::findVariableEntries(const registry::YoNamespace& namespaceValue)
{
   std::vector<YoBufferVariableEntry*> result;
   for (std::unique_ptr<YoBufferVariableEntry>& entry : entries_)
   {
      const registry::YoNamespace* entryNamespace = entry->getVariable().getNamespace();
      if (entryNamespace != nullptr && *entryNamespace == namespaceValue)
         result.push_back(entry.get());
   }
   return result;
}

std::vector<variable::YoVariable*> YoBuffer::filterVariables(const tools::VariablePredicate& filter)
{
   std::vector<YoBufferVariableEntry*> matchingEntries = filterVariableEntries(filter);
   std::vector<variable::YoVariable*> result;
   result.reserve(matchingEntries.size());
   for (YoBufferVariableEntry* entry : matchingEntries)
      result.push_back(&entry->getVariable());
   return result;
}

std::vector<YoBufferVariableEntry*> YoBuffer::filterVariableEntries(const tools::VariablePredicate& filter)
{
   std::vector<YoBufferVariableEntry*> result;
   for (std::unique_ptr<YoBufferVariableEntry>& entry : entries_)
   {
      if (filter(&entry->getVariable()))
         result.push_back(entry.get());
   }
   return result;
}

bool YoBuffer::hasUniqueVariable(const std::optional<std::string>& namespaceEnding, const std::string& name)
{
   tools::checkNameDoesNotContainSeparator(name);
   return countNumberOfEntries(namespaceEnding, name) == 1;
}

std::size_t YoBuffer::countNumberOfEntries(const std::optional<std::string>& parentNamespace, const std::string& name) const
{
   auto it = simpleNameToEntriesMap_.find(toLower(name));
   if (it == simpleNameToEntriesMap_.end() || it->second.empty())
      return 0;

   if (!parentNamespace.has_value())
      return it->second.size();

   std::size_t count = 0;
   for (YoBufferVariableEntry* entry : it->second)
   {
      const registry::YoNamespace* entryNamespace = entry->getVariable().getNamespace();
      if (entryNamespace != nullptr && entryNamespace->endsWith(*parentNamespace, true))
         count++;
   }
   return count;
}

std::string YoBuffer::toString() const
{
   std::ostringstream out;
   out << "Number of variables: " << entries_.size() << ", buffer size: " << getBufferSize() << ", in-point: " << inPoint_ << ", out-point: " << outPoint_;
   return out.str();
}
}
