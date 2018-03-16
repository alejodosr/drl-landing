#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/interprocess_semaphore.hpp>
#include <boost/python.hpp>

class RlSharedMemory {
public:
    RlSharedMemory(){
    }

    bool Open()
    {
        return true;
    }

    bool CallAction(){
        boost::interprocess::mapped_region region_;
        boost::interprocess::shared_memory_object shm_;

        // Open a shared memory object.
        shm_ = boost::interprocess::shared_memory_object
                (boost::interprocess::open_only                   //only open
                 ,"CheckActionShm"            //name
                 ,boost::interprocess::read_write                  //read-write mode
                 );

        // Map the whole shared memory in this process
        region_ = boost::interprocess::mapped_region
                (shm_                   //What to map
                 ,boost::interprocess::read_write            //Map it as read-write
                 );

        // Get the address of the mapped region
        addr_ = region_.get_address();

        //        // Construct the shared structure in memory
        shared_memory_buffer * data_ = static_cast<shared_memory_buffer*>(addr_);

        // Announce action
        data_->check_action_.post();

        return true;
    }

private:

    struct shared_memory_buffer
    {
        shared_memory_buffer()
            : check_action_(0), iterate_(0), iteration_finished_(0)
        {}

        // Semaphores to protect and synchronize access
        boost::interprocess::interprocess_semaphore
        check_action_, iterate_, iteration_finished_;

        // Items to fill
        unsigned int num_iterations;
    };

    void * addr_;
};

//char const* greet()
//{
//    boost::interprocess::mapped_region region_;
//   return "hello, world";
//}

BOOST_PYTHON_MODULE(librl_shm_python)
{
    boost::python::class_<RlSharedMemory>("RlSharedMemory", boost::python::init<>())
            //            .def_readonly("x", &RlSharedMemory::region_)
            //            //        .def("getX", boost::python::make_getter(static_cast<int RlSharedMemory::*>(&RlSharedMemory::x)))
            //            //        .def("setX", boost::python::make_setter(static_cast<int RlSharedMemory::*>(&RlSharedMemory::x)))
//            .def("Open", &RlSharedMemory::Open)
            .def("CallAction", &RlSharedMemory::CallAction)
            .def("Open", &RlSharedMemory::Open)
            ;

    //    boost::python::def("greet", greet);

}
