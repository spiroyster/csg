/*
MIT License

Copyright (c) 2021 spiroyster

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef JAHOUTF_HPP
#define JAHOUTF_HPP

#include <algorithm>
#include <exception>
#include <chrono>
#include <string>
#include <map>
#include <list>
#include <memory>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

// if c++11
// if c++14
// if gcc
#define JAHOUTF_GCC 1
// ms

#define JAHOUTF_CATCH_EXCEPTION(x) catch (const std::exception& e)\
{\
    jahoutf_current_test_->jahoutf_exception_thrown(#x, e.what());\
}\
catch (...)\
{\
    jahoutf_current_test_->jahoutf_exception_thrown(#x, "Unhandled exception!");\
}

namespace jahoutf
{
    class test;
    //class event_interface;
    //class report_interface;
    
    class report_interface
    {
    public:
        virtual void report() {}
        virtual std::string name() = 0;
    };

    class event_interface
    {
    public:
        virtual void message(const std::string& msg) {}
        virtual void case_start(const test& test) {}
        virtual void case_success(const test& test, const std::string& filename, unsigned int lineNum, const std::string& msg) {}
        virtual void case_fail(const test& test, const std::string& filename, unsigned int lineNum, const std::string& value, const std::string& expected, const std::string& tolerance, const std::string& msg) {}
        virtual void case_exception(const test& test, const std::string& msg) {}
        virtual void case_end(const test& test) {}
        virtual void suite_start() {}
        virtual void suite_end() {}
    };

    namespace _
    {
        class test_result
        {
        public:
            test_result(const std::string& g, const std::string& n) : group(g), name(n), duration_ms(0) {}
            unsigned int total() const { return success.size() + failure.size(); }
            std::string name, group, exception, exception_location;
            std::list<std::string> success, failure;
            unsigned int duration_ms;
        };
    
        class result_summary
        {
        public:
            unsigned int successful_assertions = 0, failed_assertions = 0, test_passes = 0, test_failures = 0, duration_ms = 0, exceptions_thrown = 0;
        };

        class instance
        {
        public:
            std::map<std::string, std::list<test*>> tests_;
            std::list<test_result> results_;        
            std::list<std::string> test_runner_patterns_;
            std::shared_ptr<event_interface> event_;
            std::list<std::shared_ptr<report_interface>> reports_;

            bool shuffle_ = false;
            bool list_ = false;
            std::string suite_name_;

            std::map<std::string, result_summary> summary_groups_;
            result_summary summary_;

            void summerize()
            {
                for (auto r = results_.begin(); r != results_.end(); ++r)
                {
                    result_summary& g = summary_groups_[r->group];
                    g.duration_ms = r->duration_ms;
                    g.successful_assertions += r->success.size();
                    g.failed_assertions += r->failure.size();
                    if (r->failure.empty())
                        ++g.test_passes;
                    else
                        ++g.test_failures;
                }
                for (auto g = summary_groups_.begin(); g != summary_groups_.end(); ++g)
                {
                    summary_.duration_ms += g->second.duration_ms;
                    summary_.successful_assertions += g->second.successful_assertions;
                    summary_.failed_assertions += g->second.failed_assertions;
                    summary_.test_passes += g->second.test_passes;
                    summary_.test_failures += g->second.test_failures;
                }
                results_.sort([](const test_result& a, const test_result& b)
                {
                    if (a.group == b.group)
                        return a.name < b.name;
                    return a.name < b.name;
                });
            }

            void report()
            {
                for (auto reportItr = reports_.begin(); reportItr != reports_.end(); ++reportItr)
                {
                    try 
                    { 
                        (*reportItr)->report(); 
                        event_->message("Reporting " + (*reportItr)->name() + "\n");
                    }
                    catch (...) 
                    { 
                        event_->message("Reporter (" + (*reportItr)->name() + ") threw an exception.\n"); 
                    }
                }
                results_.clear();
            }

            ~instance()
            {
                if (event_)
                    event_->suite_end();
                report();
            }

            unsigned int disabled_total();
        };
    }

    

    extern _::instance& session();

    class test
    {
    public:
        test(const std::string& group, const std::string& name) : group_(group), name_(name), result_(0)
        {
            jahoutf_current_test_ = this;
        }

        virtual void jahoutf_test_invoke()
        {
            test_result_install();
            session().event_->case_start(*this);
            begin_ = std::chrono::steady_clock::now();
            try { jahoutf_test_body(); }
            JAHOUTF_CATCH_EXCEPTION(TestBody)
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            result_->duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(begin_ - end).count();
            session().event_->case_end(*this);
        }

        virtual void jahoutf_assert_pass(const std::string& file, unsigned int lineNum, const std::string& msg)
        {
            result_->success.push_back(msg);
            session().event_->case_success(*this, file, lineNum, msg);
        }

        virtual void jahoutf_assert_fail(const std::string& filename, unsigned int lineNum, const std::string& value, const std::string& expected, const std::string& tolerance, const std::string& msg)
        {
            result_->failure.push_back(msg);
            session().event_->case_fail(*this, filename, lineNum, value, expected, tolerance, msg);
        }

        virtual void jahoutf_exception_thrown(const std::string& location, const std::string& e)
        {
            result_->exception_location = location; result_->exception = e;
            if (result_)
                result_->failure.push_back(e);
            session().event_->case_exception(*this, e);
        }

        const std::string& jahoutf_test_group() const { return group_; }
        const std::string& jahoutf_test_name() const { return name_; }
        const _::test_result& jahoutf_test_result() const { return *result_; }
        bool jahoutf_test_disabled() const { return disabled_; }
        void jahoutf_test_disabled(bool disabled) { disabled_ = disabled; }

        virtual void jahoutf_test_body() = 0;
    protected:
        test* jahoutf_current_test_;
        bool disabled_ = false;
        std::chrono::steady_clock::time_point begin_;

        void test_runner_install() { session().tests_[group_].push_back(this); }
        void test_result_install()
        {
            if (!result_)
            {
                session().results_.push_back(_::test_result(group_, name_));
                result_ = &session().results_.back();
            }
        }

        _::test_result* result_;
    private:
        std::string group_, name_;
    };


    class fixture
    {
    public:
        virtual void Setup() {}
        virtual void TearDown() {}
        virtual void prep(const void* param_value) {}
    };

    template<class T>
    class values
    {  
        std::vector<T> values_;
    public:
        values(const std::vector<T>& v) : values_(v) {}
        const std::vector<T>& get_values() const { return values_; }
    };

    template <class T>
    class fixture_param : public fixture
    {
    public:
        const T* param_value_ = 0;
        void prep(const void* p)
        {
            param_value_ = reinterpret_cast<const T*>(p);
        }
        const T& jahoutf_param() 
        {
            if (param_value_)
                return *param_value_; 
            throw std::runtime_error("Test does not support parameters."); 
        }
    };

    class section : public test
    {
    public:
        section(const std::string& group, const std::string& name) : test(group, name) 
        { 
            test_result_install();
            session().event_->case_start(*this);
            begin_ = std::chrono::steady_clock::now();
        }
        ~section() 
        { 
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            result_->duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(begin_ - end).count();
            session().event_->case_end(*this); 
        }
        void jahoutf_test_body() {}
    };


}   // namespace jahoutf

// A test case with just name...
#define JAHOUTF_TESTCLASSNAME(x) x ## instance

// Test case...
#define JAHOUTF_TEST_CASE_DEFINE(testgroup, testname) namespace testgroup \
{ \
    class testname : public jahoutf::test \
    {    \
    public: \
        testname() : jahoutf::test(#testgroup, #testname) { test_runner_install(); } \
        void jahoutf_test_body(); \
    }; \
    testname JAHOUTF_TESTCLASSNAME(testname); } \
void testgroup::testname::jahoutf_test_body()

// Test fixture...
#define JAHOUTF_TEST_FIXTURE_DEFINE(testfixture, testgroup, testname) namespace testgroup \
{ \
    class testname : public jahoutf::test \
    { \
    public:\
        class wrapper : public testfixture, public jahoutf::test\
        {\
        public:\
            wrapper() : jahoutf::test(#testgroup, #testname) {}\
            void jahoutf_test_body();\
        };\
        testname() : jahoutf::test(#testgroup, #testname) { test_runner_install(); }\
        void jahoutf_test_invoke()\
        {\
            jahoutf::_::test_result r(jahoutf_test_group(), jahoutf_test_name());\
            result_ = &r;\
            try\
            { \
                wrapper w;\
                try \
                { \
                    w.Setup();\
                    w.jahoutf_test_invoke();\
                    try { w.TearDown(); } \
                    JAHOUTF_CATCH_EXCEPTION(fixture_teardown)\
                }\
                JAHOUTF_CATCH_EXCEPTION(fixture_setup)\
            }\
            JAHOUTF_CATCH_EXCEPTION(fixture_construct)\
        }\
        void jahoutf_test_body() {}\
    };\
    testname JAHOUTF_TESTCLASSNAME(testname); }\
void testgroup::testname::wrapper::jahoutf_test_body()

#define JAHOUTF_TESTPARAMSNAME(testgroup, testname) testname ## testgroup ## _

// Test case values...
#define JAHOUTF_TEST_CASE_VALUES_DEFINE(testgroup, testname, testvalues) namespace testgroup\
{\
    auto JAHOUTF_TESTPARAMSNAME(testgroup, testname) = testvalues;\
    class testname : public jahoutf::test\
    {\
    public:\
        class wrapper : public jahoutf::test\
        {\
        public:\
            wrapper(unsigned int n) : jahoutf::test(#testgroup, #testname + std::string("[") + std::to_string(n) + std::string("]")), param_id_(n) {}\
            void jahoutf_test_body();\
            const auto& jahoutf_value() { return JAHOUTF_TESTPARAMSNAME(testgroup, testname).get_values()[param_id_]; }\
            unsigned int param_id_;\
        };\
        testname() : jahoutf::test(#testgroup, #testname) { test_runner_install(); }\
        void jahoutf_test_invoke()\
        {\
            for (unsigned int p = 0; p < JAHOUTF_TESTPARAMSNAME(testgroup, testname).get_values().size(); ++p)\
            {\
                try\
                {\
                    wrapper w(p);\
                    w.jahoutf_test_invoke();\
                }\
                JAHOUTF_CATCH_EXCEPTION(test_body)\
            }\
        }\
        void jahoutf_test_body() {}\
    };\
    testname JAHOUTF_TESTCLASSNAME(testname);\
}\
void testgroup::testname::wrapper::jahoutf_test_body()

// Test fixture values... TEST_F_VALUES
#define JAHOUTF_TEST_FIXTURE_VALUES_DEFINE(testgroup, testname, testfixture, testvalues) namespace testgroup\
{\
    auto JAHOUTF_TESTPARAMSNAME(testgroup, testname) = testvalues;\
    class testname : public jahoutf::test\
    {\
    public:\
        class wrapper : public jahoutf::test, public testfixture \
        {\
        public:\
            wrapper(unsigned int n) : jahoutf::test(#testgroup, #testname + std::string("[") + std::to_string(n) + std::string("]")), param_id_(n) {}\
            void jahoutf_test_body();\
            const auto& jahoutf_value() { return JAHOUTF_TESTPARAMSNAME(testgroup, testname).get_values()[param_id_]; }\
            unsigned int param_id_;\
        };\
        testname() : jahoutf::test(#testgroup, #testname) { test_runner_install(); }\
        void jahoutf_test_invoke()\
        {\
            for (unsigned int p = 0; p < JAHOUTF_TESTPARAMSNAME(testgroup, testname).get_values().size(); ++p)\
            {\
                try\
                { \
                    wrapper w(p);\
                    w.prep(&JAHOUTF_TESTPARAMSNAME(testgroup, testname).get_values()[p]);\
                    try \
                    { \
                        w.Setup();\
                        w.jahoutf_test_invoke();\
                        try { w.TearDown(); } \
                        JAHOUTF_CATCH_EXCEPTION(fixture_teardown)\
                    }\
                    JAHOUTF_CATCH_EXCEPTION(fixture_setup)\
                }\
                JAHOUTF_CATCH_EXCEPTION(fixture_construct)\
            }\
        }\
        void jahoutf_test_body() {}\
    };\
    testname JAHOUTF_TESTCLASSNAME(testname);\
}\
void testgroup::testname::wrapper::jahoutf_test_body()

#define JAHOUTF_TEST_SECTION_DEFINE(testgroup, testname) if (std::unique_ptr<jahoutf::section> jahoutf_current_test_ = std::make_unique<jahoutf::section>(#testgroup, #testname))









// default xUnit and Console output...
namespace jahoutf
{
    class console : public event_interface
    {
        public:
        
        std::string Black(const std::string& msg) { return std::string("\033[1;30m" + msg + "\033[0m"); }
        std::string Red(const std::string& msg) { return std::string("\033[1;31m" + msg + "\033[0m"); }
        std::string Green(const std::string& msg) { return std::string("\033[1;32m" + msg + "\033[0m"); }
        std::string Yellow(const std::string& msg) { return std::string("\033[1;33m" + msg + "\033[0m"); }
        std::string Blue(const std::string& msg) { return std::string("\033[1;34m" + msg + "\033[0m"); }
        std::string Magenta(const std::string& msg) { return std::string("\033[1;35m" + msg + "\033[0m"); }
        std::string Cyan(const std::string& msg) { return std::string("\033[1;36m" + msg + "\033[0m"); }
        std::string White(const std::string& msg) { return std::string("\033[0;37m" + msg + "\033[0m"); }
        std::string Inverse(const std::string& msg) { return std::string("\033[1;7m" + msg + "\033[0m"); }

        std::string header(const test& test) { return std::string("\n" + test.jahoutf_test_group() + "." + test.jahoutf_test_name() + " "); }
        std::string duration(unsigned int d) { return std::string("(" + std::to_string(d) + "ms)"); }
        std::string results_bar(unsigned int successes, unsigned int failures)
        {
            unsigned int total = successes + failures, length = 20;
            float percentage = static_cast<float>(failures) / static_cast<float>(total);
            unsigned int f_bar = static_cast<int>(percentage * static_cast<float>(length));
            if (!f_bar && failures) { f_bar = 1; }
            unsigned int s_bar = length - f_bar;
            std::ostringstream oss;
            oss << "|" << Green(std::string(s_bar, '=')) << Red(std::string(f_bar, '=')) << "| ";
            return oss.str();
        }
        virtual void message(const std::string& msg) { std::cout << msg; }
        virtual void case_success(const test& test, const std::string& filename, unsigned int lineNum, const std::string& msg) { if (!msg.empty()) { std::cout << header(test) << Cyan(filename + ":") << Yellow(std::to_string(lineNum)) << msg << "\n"; } }
        virtual void case_fail(const test& test, const std::string& filename, unsigned int lineNum, const std::string& value, const std::string& expected, const std::string& tolerance, const std::string& msg) 
        { 
            std::cout << header(test) << Red("fail ") << Cyan(filename + ":") << Yellow(std::to_string(lineNum)); 
            if (!value.empty())
                std::cout << "\nValue     | " << White(value);
            if (!expected.empty())
                std::cout << "\nExpected  | " << White(expected);
            if (!tolerance.empty())
                std::cout << "\nTolerance | " << White(tolerance);
            if (!msg.empty()) 
                std::cout << msg << "\n";
        }
        virtual void case_exception(const test& test, const std::string& exc)  { std::cout << header(test) << Magenta("EXCEPTION ") << "thrown in " << test.jahoutf_test_result().exception_location << "\n" << exc; }
        virtual void case_end(const test& test) 
        {
            std::cout << header(test);
            if (test.jahoutf_test_result().failure.empty())
                std::cout << Green(" PASSED ") << std::to_string(test.jahoutf_test_result().success.size()) << "/" << std::to_string(test.jahoutf_test_result().total()) << " assertions succeeded. " << duration(test.jahoutf_test_result().duration_ms) << '\n';
            else
                std::cout << Red(" FAILED ") << std::to_string(test.jahoutf_test_result().failure.size()) << "/" << std::to_string(test.jahoutf_test_result().total()) << " assertions failed.\n";
        }
        virtual void suite_start() 
        {
            unsigned int test_count = 0, group_count = 0, disabled_count = 0;
            for (auto g = session().tests_.begin(); g != session().tests_.end(); ++g, ++group_count)
            {
                test_count += g->second.size();
                for (auto t = g->second.begin(); t != g->second.end(); ++t)
                    disabled_count += (*t)->jahoutf_test_disabled() ? 1 : 0;
            }
            std::cout << Inverse("Running " + std::to_string(test_count - disabled_count) + " tests (" + std::to_string(group_count) + " groups)") << " (type \"?\" for help)\n";
            if (session().shuffle_)
                std::cout << Cyan(" [SHUFFLE] ");
            if (disabled_count)
                std::cout << Yellow(" [" + std::to_string(disabled_count) + " tests are disabled]");
        }
        virtual void suite_end() 
        {
            std::cout << "\n" << Inverse("Results");
            for (auto g = session().summary_groups_.begin(); g != session().summary_groups_.end(); ++g)
            {
                std::cout << "\n" << results_bar(g->second.test_passes, g->second.test_failures) << " {" << Cyan(g->first) << "} ";
                std::cout << std::to_string(g->second.test_passes) << "/" << std::to_string(g->second.test_passes + g->second.test_failures) << " tests passed ";
                std::cout << "(" << std::to_string(g->second.successful_assertions) << "/" << std::to_string(g->second.successful_assertions + g->second.failed_assertions) << " assertions).";
                std::cout << duration(g->second.duration_ms);    
            }
            std::cout << "\n" << Inverse("Total");
            std::cout << "\n" << results_bar(session().summary_.test_passes, session().summary_.test_failures);
            std::cout << std::to_string(session().summary_.test_passes) << "/" << std::to_string(session().summary_.test_passes + session().summary_.test_failures) << " tests passed ";
            std::cout << "(" << std::to_string(session().summary_.successful_assertions) << "/" << std::to_string(session().summary_.successful_assertions + session().summary_.failed_assertions) << " assertions).";
            std::cout << duration(session().summary_.duration_ms) << "\n";
        }
    };

    class xUnit : public jahoutf::report_interface
    {
        static void replace_all(std::string& str, const std::string& what, const std::string& with)
		{
			std::size_t itr = str.find(what);
			while (itr != std::string::npos)
			{
				str.replace(itr, itr + what.size(), with.c_str());
				itr = str.find(what);
			}
		}
		static std::string escape_xml_chars(const std::string& syntaxToConvert)
		{
			std::string convertedSyntax = syntaxToConvert;
			replace_all(convertedSyntax, ">", "&gt;");
			replace_all(convertedSyntax, "<", "&lt;");
			replace_all(convertedSyntax, "&", "&amp;");
			replace_all(convertedSyntax, "'", "&apos;");
			replace_all(convertedSyntax, "\"", "&quot;");
			return convertedSyntax;
		}

        class testsuite_raii
        {
            std::ostringstream& oss_;
            std::string close_syntax_;
        public:
            testsuite_raii(std::ostringstream& oss, const std::string& name, const jahoutf::_::result_summary& results, unsigned int disabled_count, const std::string& indent)
            :   oss_(oss)
            {
                oss_ << "\n" << indent << "<testsuite name=\"" << name << "\" tests=\"" << (results.test_passes + results.test_failures) << "\" failures=\"" << results.test_failures << "\" disabled=\"" << disabled_count << "\" errors=\"" << results.exceptions_thrown << "\" time=\"" << results.duration_ms << "\">";
                close_syntax_ = std::string("\n" + indent + "</testsuite>\n");
            }
            ~testsuite_raii()
            {
                oss_ << close_syntax_;
            }
        };

        class testcase_raii
        {
            std::ostringstream& oss_;
            std::string close_syntax_;
        public:
            testcase_raii(std::ostringstream& oss, const jahoutf::_::test_result& result, const std::string& indent)
            :   oss_(oss)
            {
                oss_ << "\n" << indent << "<testcase name=\"" << result.name << "\" status=\"run\" time=\"" << result.duration_ms << "\" classname=\"" << result.group << "\">";
                for (auto f = result.failure.begin(); f != result.failure.end(); ++f)
                    oss_ <<  "\n" << indent << "  " << "<failure message=\"" << xUnit::escape_xml_chars(*f) << "\"></failure>";
                oss_ << "\n" << indent << "</testcase>\n";
            }
            ~testcase_raii()
            {
                oss_ << close_syntax_;
            }
        };




    public:
        std::string filepath_;
        xUnit(const std::string& filename) : filepath_(filename) {}
        std::string name() { return std::string("xUnit xml (" + filepath_ + ")"); }

        void report()
        {
            unsigned int test_count = 0, disabled_count = 0;
            for (auto g = session().tests_.begin(); g != session().tests_.end(); ++g)
            {
                test_count += g->second.size();
                for (auto t = g->second.begin(); t != g->second.end(); ++t)
                    disabled_count += (*t)->jahoutf_test_disabled() ? 1 : 0;
            }
            std::ostringstream oss;
            oss << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
            testsuite_raii ts(oss, session().suite_name_, session().summary_, disabled_count, "");
            if (!session().results_.empty())
            {
                std::string group = session().results_.begin()->group;
                auto g = std::make_shared<testsuite_raii>(oss, group, session().summary_groups_[group], 0, "  ");
                for (auto r = session().results_.begin(); r != session().results_.end(); ++r)
                {
                    if (r->group != group)
                    {
                        group = r->group;
                        g = std::make_shared<testsuite_raii>(oss, group, session().summary_groups_[group], 0, "  ");
                    }
                    testcase_raii t(oss, *r, "    ");
                }
            }
            std::ofstream file(filepath_);
            if (file)
                file << oss.str();
            else
                throw std::runtime_error("Unable to open file " + filepath_);
        }
    };


    // test runner functions...
    namespace _
    {
        static bool test_runner_pattern_match(const jahoutf::test& t)
        {
            for (auto patternItr = session().test_runner_patterns_.begin(); patternItr != session().test_runner_patterns_.end(); ++patternItr)
            {
                std::size_t seperator = patternItr->find(".");
                std::string group = patternItr->substr(0, seperator);
                std::string name = seperator == std::string::npos ? "" : patternItr->substr(seperator+1);
                
                if (group == "*")
                    return name == t.jahoutf_test_name();
                if (name == "*")
                    return group == t.jahoutf_test_group();
                if (name == t.jahoutf_test_name() && group == t.jahoutf_test_group())
                    return true;
            }
            return false;
        }

        static void test_runner_list_tests()
        {
            for (auto groupItr = session().tests_.begin(); groupItr != session().tests_.end(); ++groupItr)
                for (auto testItr = groupItr->second.begin(); testItr != groupItr->second.end(); ++testItr)
                    session().event_->message(groupItr->first + "." + (*testItr)->jahoutf_test_name() + "\n");
        }

        static void test_runner_disable_all_tests()
        {
            for (auto groupItr = session().tests_.begin(); groupItr != session().tests_.end(); ++groupItr)
                for (auto testItr = groupItr->second.begin(); testItr != groupItr->second.end(); ++testItr)
                    (*testItr)->jahoutf_test_disabled(true);
        }

        static void test_runner_run_tests()
        {  
            // If specified, list the tests and leave...
            if (session().list_)
            {
                test_runner_list_tests();
                return;
            }

            // Process any pattern matching we need to do...
            if (!session().test_runner_patterns_.empty())
            {
                test_runner_disable_all_tests();
                for (auto groupItr = session().tests_.begin(); groupItr != session().tests_.end(); ++groupItr)
                    for (auto testItr = groupItr->second.begin(); testItr != groupItr->second.end(); ++testItr)
                        if (test_runner_pattern_match(**testItr))
                            (*testItr)->jahoutf_test_disabled(false);
            }

            // Start the session...
            session().event_->suite_start();
            unsigned int tests_count = 0, tests_passed = 0, tests_failed = 0;
            for (auto groupItr = session().tests_.begin(); groupItr != session().tests_.end(); ++groupItr)
            {
                //if (session().shuffle_)
                 //   std::random_shuffle(groupItr->second.begin(), groupItr->second.end());
                for (auto testItr = groupItr->second.begin(); testItr != groupItr->second.end(); ++testItr)
                {
                    if (!(*testItr)->jahoutf_test_disabled())
                        (*testItr)->jahoutf_test_invoke();
                }    
            } 
            session().summerize(); 
            session().event_->suite_end();
        }

        static std::string test_runner_help_message()
        {
            std::ostringstream oss;
            oss << "jahoutf test runner.\n";
            oss << "Usage: > " << session().suite_name_ << " [-silent] [-list] [-shuffle] [-xunit=\"filename.xml\"] [?] test1 test2 ...\n";
            oss << "\n";
            oss << "-list    : list the tests in this test program.\n";
            oss << "-shuffle : shuffle the tests before running them.\n";
            oss << "-silent  : silence events. No consol output.\n";
            oss << "-xunit   : specify an xml filename to report xUnit to.\n";
            oss << "\n";
            oss << "If one or more tests are specfied to run, all tests except those matching the name patterns will be disabled.\n";
            oss << "To run a single test, use {groupname}.{testname} format (i.e group and name seperated by dot).\n";
            return oss.str();
        }

        static void test_runner_process_arguments(const std::vector<std::string>& args)
        {
            session().suite_name_ = args.front();
            for (unsigned int arg = 1; arg < args.size(); ++arg)
            {
                if (args[arg] == "-silent")
                    jahoutf::session().event_.reset(new event_interface());
                else if (args[arg] == "-shuffle")
                    jahoutf::session().shuffle_ = true;
                else if (args[arg] == "-list")
                    jahoutf::session().list_ = true;
                else if (args[arg] == "?")
                {
                    jahoutf::session().event_->message(test_runner_help_message());
                    jahoutf::session().tests_.clear();
                }
                else
                {
                    std::size_t xu = args[arg].find("-xunit=");
                    if (xu == std::string::npos)
                        jahoutf::session().test_runner_patterns_.push_back(args[arg]);    
                    else
                        jahoutf::session().reports_.push_back(std::make_shared<jahoutf::xUnit>(args[arg].substr(xu + 7)));
                }
                    
            }
        }
    }

    



}   // namespace jahoutf

// Macro to initialise jahoutf when using custom main entry func... only needs to be used if not using JAHOUTF_MAIN and instead uses own custom main function.
//  jahoutf_XXX arguments not supported with 
#define JAHOUTF_INSTANCE namespace jahoutf \
{ \
    namespace _ \
    { \
        static std::unique_ptr<instance> session_; \
    } \
    _::instance& session() \
    { \
        if (!_::session_) \
        { \
            _::session_.reset(new _::instance()); \
            _::session_->event_.reset(new console()); \
        } \
        return *_::session_; \
    } \
}

// namespace jahoutf
// {
//     namespace _
//     {
//         instance::~instance() 
//         {
//             if (event_)
//                 event_->suite_end();
//             report();
//         }
//     }
// }

// instance();
//             ~instance();

// Macro for defining main function, user can perform custom global setup and teardown (applied to entire test program) also processed -jahoutf arguments.
// Must explicitly call RUNALL, RUNALL_RANDOMIZED or explicit tests?
#define JAHOUTF_TEST_RUNNER JAHOUTF_INSTANCE \
void test_runner_main(); \
int main(int argc, char** argv) \
{ \
    std::vector<std::string> args; \
    for (unsigned int c = 0; c < argc; ++c) { args.push_back(std::string(*(argv+c))); }\
    jahoutf::_::test_runner_process_arguments(args);\
    if (!jahoutf::session().tests_.empty()) { test_runner_main(); } \
    return 0; \
} \
void test_runner_main()

// Run all the tests.... used if user has global startup and teardown functionality in main....
#define RUNALL jahoutf::_::test_runner_run_tests();
// Shuffle all the tests before running them (not applicable to section/inline tests)...
#define SHUFFLE jahoutf::session().shuffle_ = true; RUNALL
// Silence all the events
#define SILENT jahoutf::session().event_.reset(new jahoutf::_::instance::event_interface());
// Maybe post? Post current results to report...
//#define JAHOUTF_REPORT(x) std::shared_ptr<x>    jahoutf::session().reports_.push_back(std::shared_ptr<new )
//#define JAHOUTF_EVENT(x) x
#define JAHOUTF_POST jahoutf::session().report();

#define TEST_CASE_1(testgroup, testname) JAHOUTF_TEST_CASE_DEFINE(testgroup, testname)
#define TEST_CASE_2(testname) JAHOUTF_TEST_CASE_DEFINE(, testname)
#define GET_TEST_CASE_MACRO(_1,_2,TCNAME,...) TCNAME
#define TEST(...) GET_TEST_CASE_MACRO(__VA_ARGS__, TEST_CASE_1, TEST_CASE_2)(__VA_ARGS__)

#define TEST_SECTION_1(testgroup, testname) JAHOUTF_TEST_SECTION_DEFINE(testgroup, testname)
#define TEST_SECTION_2(testname) JAHOUTF_TEST_SECTION_DEFINE(, testname)
#define GET_TEST_SECTION_MACRO(_1,_2,TSNAME,...) TSNAME
#define TEST_S(...) GET_TEST_SECTION_MACRO(__VA_ARGS__, TEST_SECTION_1, TEST_SECTION_2)(__VA_ARGS__)

#define TEST_FIXTURE_1(testgroup, testname, testfixture) JAHOUTF_TEST_FIXTURE_DEFINE(testfixture, testgroup, testname)
#define TEST_FIXTURE_2(testname, testfixture) JAHOUTF_TEST_FIXTURE_DEFINE(testfixture,, testname)
#define GET_TEST_FIXTURE_MACRO(_1,_2,_3, TFNAME,...) TFNAME
#define TEST_F(...) GET_TEST_FIXTURE_MACRO(__VA_ARGS__, TEST_FIXTURE_1, TEST_FIXTURE_2)(__VA_ARGS__)

#define TEST_VALUES_1(testgroup, testname, testvalues) JAHOUTF_TEST_CASE_VALUES_DEFINE(testgroup, testname, testvalues)
#define TEST_VALUES_2(testname, testvalues) JAHOUTF_TEST_CASE_VALUES_DEFINE(,testname, testvalues)
#define GET_TEST_VALUES_MACRO(_1,_2,_3, TVNAME,...) TVNAME
#define TEST_VALUES(...) GET_TEST_VALUES_MACRO(__VA_ARGS__, TEST_VALUES_1, TEST_VALUES_2)(__VA_ARGS__)

#define TEST_F_VALUES_1(testgroup, testname, testfixture, testvalues) JAHOUTF_TEST_FIXTURE_VALUES_DEFINE(testgroup, testname, testfixture, testvalues)
#define TEST_F_VALUES_2(testname, testfixture, testvalues) JAHOUTF_TEST_FIXTURE_VALUES_DEFINE(,testname, testfixture, testvalues)
#define GET_TEST_F_VALUES_MACRO(_1,_2,_3,_4, TFVNAME,...) TFVNAME
#define TEST_F_VALUES(...) GET_TEST_F_VALUES_MACRO(__VA_ARGS__, TEST_F_VALUES_1, TEST_F_VALUES_2)(__VA_ARGS__)


// Assertions...
#define SUCCESS jahoutf_current_test_->jahoutf_assert_pass(__FILE__, __LINE__, "");
#define FAIL jahoutf_current_test_->jahoutf_assert_fail(__FILE__, __LINE__, "", "", "", "");
#define EXPECT_EQ(a, b) if (a == b) { SUCCESS } else { jahoutf_current_test_->jahoutf_assert_fail(__FILE__, __LINE__, #a, #b, "", ""); };
#define EXPECT(a) EXPECT_EQ(a, true);
#define EXPECT_NEAR(a, b, e) if (abs(b-a) <= e) { SUCCESS } else { jahoutf_current_test_->jahoutf_assert_fail(__FILE__, __LINE__, #a, #b, #e, ""); };
// EXPECT_THROW
// ASSERT_THAT

#endif // JAHOUTF_HPP