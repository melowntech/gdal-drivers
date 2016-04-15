#include <boost/format.hpp>
#include <boost/filesystem/path.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/streams.hpp"
#include "utility/binaryio.hpp"
#include "service/cmdline.hpp"

#include "geo/geodataset.hpp"
#include "gdal-drivers/register.hpp"
#include "gdal-drivers/borderedarea.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace bin = utility::binaryio;

class Ba2Mask : public service::Cmdline {
public:
    Ba2Mask()
        : service::Cmdline("ba2mask", "test")
    {}

private:
    void configuration(po::options_description &cmdline
                       , po::options_description &config
                       , po::positional_options_description &pd);

    void configure(const po::variables_map&) {}

    int run();

    fs::path in_;
    fs::path out_;
};

void Ba2Mask::configuration(po::options_description &cmdline
                           , po::options_description &config
                           , po::positional_options_description &pd)
{
    config.add_options()
        ("in", po::value(&in_)->required()
         , "Path to input dataset.")
        ("out", po::value(&out_)->required()
         , "Path to output mask.")
        ;

    (void) cmdline;
    (void) pd;
}

int Ba2Mask::run()
{
    gdal_drivers::BorderedAreaDataset::asMask(in_, out_);
    return EXIT_SUCCESS;
}

int main(int argc, char *argv[])
{
    gdal_drivers::registerAll();
    return Ba2Mask()(argc, argv);
}
