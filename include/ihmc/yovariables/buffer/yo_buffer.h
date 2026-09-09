#pragma once

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "ihmc/yovariables/buffer/interfaces/yo_buffer_index_changed_listener.h"
#include "ihmc/yovariables/buffer/interfaces/yo_buffer_processor.h"
#include "ihmc/yovariables/buffer/interfaces/yo_buffer_reader.h"
#include "ihmc/yovariables/buffer/interfaces/yo_buffer_variable_entry_holder.h"
#include "ihmc/yovariables/buffer/interfaces/yo_time_buffer_holder.h"
#include "ihmc/yovariables/buffer/key_points_handler.h"
#include "ihmc/yovariables/buffer/yo_buffer_variable_entry.h"
#include "ihmc/yovariables/registry/yo_variable_holder.h"

namespace ihmc::yovariables::variable
{
class YoVariable;
}

namespace ihmc::yovariables::buffer
{
/**
 * Manages buffers to store history for a collection of YoVariables.
 * <p>
 * When adding a variable to this buffer via addVariable(), a new YoBufferVariableEntry for that
 * new variable is created.
 * </p>
 * <p>
 * Once variables have been registered, the two main methods of interest are:
 * <ul>
 * <li>tickAndWriteIntoBuffer() to take a single step in the buffer, then write the YoVariable
 * values into the buffer.
 * <li>tickAndReadFromBuffer(int) to step forward or backward in the buffer, then read the buffer
 * and update the YoVariables.
 * <li>setCurrentIndex(int) to change the current position in the buffer, then read the buffer and
 * update the YoVariables.
 * </ul>
 * </p>
 * <p>
 * Ownership: unlike YoVariable/YoRegistry (which store non-owning back-pointers, mirroring Java's
 * reference semantics, since the caller that constructs them owns their lifetime), YoBuffer owns
 * its YoBufferVariableEntry instances outright - they are internal bookkeeping objects created and
 * destroyed entirely within this class's own lifecycle, with no other stated owner in the Java
 * source. Entries are held via std::unique_ptr accordingly.
 * </p>
 */
class YoBuffer : public registry::YoVariableHolder,
                  public interfaces::YoBufferReader,
                  public interfaces::YoTimeBufferHolder,
                  public interfaces::YoBufferVariableEntryHolder
{
public:
   explicit YoBuffer(int bufferSize);

   /** Clone constructor. */
   explicit YoBuffer(const YoBuffer& other);
   YoBuffer& operator=(const YoBuffer&) = delete;

   /** Clears the internal data and removes all variables and their buffers. */
   void clear();

   /** Clears the internal buffers (overwriting previously written data) and resizes this buffer. */
   void clearBuffers(int bufferSize);

   /**
    * Resizes this buffer, preserving the data in [inPoint, outPoint] when possible (shifted to the
    * beginning of the buffer). If newBufferSize is too small to retain that data, the buffer is
    * cropped to [inPoint, inPoint + newBufferSize - 1].
    */
   void resizeBuffer(int newBufferSize);

   void setLockIndex(bool lock);
   bool isIndexLocked() const;

   /** Adds entry to this buffer. entry's buffer size must match this buffer's size. */
   void addEntry(std::unique_ptr<YoBufferVariableEntry> entry);

   /**
    * Registers variable to this buffer, associating it with a new buffer entry. If already
    * registered, does nothing and returns the existing entry.
    */
   YoBufferVariableEntry& addVariable(variable::YoVariable& variable);

   /** Registers a list of variables; already-registered ones are skipped. */
   void addVariables(const std::vector<variable::YoVariable*>& variables);

   /** Removes variable from this buffer. Returns the removed entry's owning pointer, or nullptr. */
   std::unique_ptr<YoBufferVariableEntry> removeVariable(variable::YoVariable& variable);

   void setInPoint(int index);
   void setOutPoint(int index);
   void setInPoint();
   void setOutPoint();
   void setInOutPointFullBuffer();

   /** Toggles a key point at the current index. */
   void toggleKeyPoint();

   bool isAtInPoint() const;
   bool isAtOutPoint() const;

   /** Reads the buffer at the current index and updates the values of the variables. */
   void readFromBuffer();

   /** Writes into the buffer at the current index the current values of the variables. */
   void writeIntoBuffer();

   void gotoInPoint();
   void gotoOutPoint();

   void setCurrentIndex(int index) override;
   bool tickAndReadFromBuffer(int stepSize) override;

   /** Increments the current buffer index and writes the variables' values into the buffer there. */
   void tickAndWriteIntoBuffer();

   /** Fills the buffer with the current values of the variables. */
   void fillBuffer();

   /** Shifts the data in the buffer such that the in-point is at 0. */
   void shiftBuffer();
   void shiftBuffer(int shiftIndex);

   /** Crops the buffer to retain only [inPoint, outPoint]. */
   void cropBuffer();
   void cropBuffer(int start, int end);

   /** Cuts the buffer, removing [inPoint, outPoint]. */
   void cutBuffer();
   void cutBuffer(int start, int end);

   /** Prunes data, keeping only every n-th point; divides this buffer's size by n. */
   void thinData(int n);

   /** The average value for variable over its entire buffer, or NaN if not found. */
   double computeAverage(variable::YoVariable& variable) const;

   /** Applies a processor throughout the buffer to read and/or modify this buffer. */
   void applyProcessor(interfaces::YoBufferProcessor& processor);

   int getInPoint() const override;
   int getOutPoint() const override;

   /** Next key point index closest to the current buffer index. */
   int getNextKeyPoint() const;
   /** Previous key point index closest to the current buffer index. */
   int getPreviousKeyPoint() const;

   int getBufferSize() const override;

   /** Covariant override: YoBufferVariableEntry publicly implements YoBufferVariableEntryReader. */
   YoBufferVariableEntry* getEntry(variable::YoVariable& variable) override;

   /** The buffer variable entries managed by this buffer. */
   const std::vector<std::unique_ptr<YoBufferVariableEntry>>& getEntries() const;

   std::vector<variable::YoVariable*> getVariables() override;

   void addListener(interfaces::YoBufferIndexChangedListener* listenerToAdd);
   void removeListeners();
   bool removeListener(interfaces::YoBufferIndexChangedListener* listenerToRemove);

   int getCurrentIndex() const override;

   /**
    * Tests whether this buffer and other are equal to an epsilon: same [inPoint, outPoint] length,
    * same set of variables (paired by full name), and equal data within that interval.
    */
   bool epsilonEquals(const YoBuffer& other, double epsilon) const;

   /**
    * Sets the name of the variable that represents time. Does nothing (and logs nothing - unlike
    * the Java source, which logs via a project-specific logger not part of this port) if no
    * variable with that name is registered.
    */
   void setTimeVariableName(const std::string& timeVariableName);
   const std::string& getTimeVariableName() const;

   KeyPointsHandler& getKeyPointsHandler();

   std::vector<double> getTimeBuffer() const override;

   variable::YoVariable* findVariable(const std::optional<std::string>& namespaceEnding, const std::string& name) override;
   YoBufferVariableEntry* findVariableEntry(const std::string& name);
   YoBufferVariableEntry* findVariableEntry(const std::optional<std::string>& namespaceEnding, const std::string& name);

   std::vector<variable::YoVariable*> findVariables(const std::optional<std::string>& namespaceEnding, const std::string& name) override;
   std::vector<YoBufferVariableEntry*> findVariableEntries(const std::string& name);
   std::vector<YoBufferVariableEntry*> findVariableEntries(const std::optional<std::string>& namespaceEnding, const std::string& name);

   std::vector<variable::YoVariable*> findVariables(const registry::YoNamespace& namespaceValue) override;
   std::vector<YoBufferVariableEntry*> findVariableEntries(const registry::YoNamespace& namespaceValue);

   std::vector<variable::YoVariable*> filterVariables(const tools::VariablePredicate& filter) override;
   std::vector<YoBufferVariableEntry*> filterVariableEntries(const tools::VariablePredicate& filter);

   bool hasUniqueVariable(const std::optional<std::string>& namespaceEnding, const std::string& name) override;

   using YoVariableHolder::findVariable;
   using YoVariableHolder::findVariables;
   using YoVariableHolder::hasUniqueVariable;
   using YoVariableHolder::hasVariable;

   std::string toString() const;

private:
   void enlargeBufferSize(int newBufferSize);
   void notifyIndexChangedListeners();
   std::size_t countNumberOfEntries(const std::optional<std::string>& parentNamespace, const std::string& name) const;

   std::string timeVariableName_ = "t";

   int inPoint_ = 0;
   int outPoint_ = 0;
   int currentIndex_ = 0;
   int bufferSize_;

   std::vector<std::unique_ptr<YoBufferVariableEntry>> entries_;
   std::unordered_map<std::string, std::vector<YoBufferVariableEntry*>> simpleNameToEntriesMap_;
   KeyPointsHandler keyPointsHandler_;
   std::vector<interfaces::YoBufferIndexChangedListener*> indexChangedListeners_;

   bool lockIndex_ = false;
};
}
