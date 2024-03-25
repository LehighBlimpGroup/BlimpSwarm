


#include "state/IConfig.h"
#include <Preferences.h>

// another_t definition
typedef struct {
    int mode;
    double volume;
} custom_t;

// Specific configuration class for another_t
class CustomConfig : public IConfig<custom_t> {
private:
    custom_t configData;

public:
    void loadConfiguration() override {
        // Assume this loads from a different source, like a file, or different preferences
    }

    const custom_t& getConfiguration() const override {
        return configData;
    }
};
