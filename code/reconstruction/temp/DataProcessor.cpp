#include "DataProcessor.h"

/*virtual*/ void DataProcessor::onRead(Vector &text) {
	printf("Got %s\n", text.toString().c_str());
}
