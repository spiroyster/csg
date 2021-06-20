#ifndef LOGIQA_HPP
#define LOGIQA_HPP

// Stl includes... C++14
#include <algorithm>
#include <map>
#include <vector>
#include <memory>
#include <string>
#include <chrono>

// Unless specified, by default include default console...
#ifndef LOGIQA_DEFAULT_CONSOLE_EXCLUDE
#define LOGIQA_INCLUDE_DEFAULT_CONSOLE
#endif

// Unless specified, by default include default xUnit...
#ifndef LOGIQA_DEFAULT_XUNIT_EXCLUDE
#define LOGIQA_INCLUDE_DEFAULT_XUNIT
#endif

// LOGIQA_CONSOLE_COLOUR_DISABLED

// Check compiler...
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
#define LOGIQA_WIN 1
#include <Windows.h>
#elif defined(__linux__) || defined(__unix__)
#define LOGIQA_NIX
#define LOGIQA_ANSI 1
#endif


// Macros defining the tests...

#define LOGIQA_TEST(name, tags) namespace logiqa { namespace tests { namespace name \
{ \
	class test_runner_wrapper : public logiqa::test \
	{ \
	public: \
		test_runner_wrapper() : logiqa::test(#name, 0, tags) { session().tests_[logiqa_unique_name()] = this; } \
		void logiqa_body() override; \
	}; \
	static test_runner_wrapper trw; \
} } }\
void logiqa::tests::name::test_runner_wrapper::logiqa_body()

// Test array of parameter values...
#define LOGIQA_TEST_PARAMS(name, tags, params) namespace logiqa { namespace tests { namespace name \
{ \
	class test_runner_wrapper : public logiqa::test \
	{ \
	public: \
		test_runner_wrapper() : logiqa::test("", 0, "") {} \
		test_runner_wrapper(int index) : logiqa::test(#name, index, tags) { session().tests_[logiqa_unique_name()] = this; } \
		const auto& logiqa_param() { return params.get_values()[logiqa_param_num()]; } \
		void logiqa_body() override; \
	}; \
	static std::vector<std::shared_ptr<test_runner_wrapper>> instances() \
	{ \
		std::vector<std::shared_ptr<test_runner_wrapper>> result; \
		for (int p = 0; p < params.get_values().size(); ++p) \
			result.push_back(std::make_shared<test_runner_wrapper>(p)); \
		return result; \
	} \
	static std::vector<std::shared_ptr<test_runner_wrapper>> votcw = instances(); \
} } }\
void logiqa::tests::name::test_runner_wrapper::logiqa_body()

// Test a single fixture
#define LOGIQA_TEST_FIXTURE(name, tags, fixture) namespace logiqa { namespace tests { namespace name \
{ \
	class test_runner_wrapper : public logiqa::test, public fixture \
	{ \
	public: \
		test_runner_wrapper() : logiqa::test(#name, 0, tags) { session().tests_[logiqa_unique_name()] = this; } \
		void logiqa_body() override; \
		void logiqa_run() override \
		{ \
			try { setup(); } \
			LOGIQA_EXCEPTION_CATCHER(fixture_setup) \
			test::logiqa_run(); \
			try { teardown(); } \
			LOGIQA_EXCEPTION_CATCHER(fixture_teardown) \
		} \
	}; \
	static test_runner_wrapper trw; \
} } }\
void logiqa::tests::name::test_runner_wrapper::logiqa_body()

// Test a single fixture who has access to the params...
#define LOGIQA_TEST_FIXTURE_PARAMS(name, tags, fixture, params) namespace logiqa { namespace tests { namespace name \
{ \
	class test_runner_wrapper : public logiqa::test, public fixture \
	{ \
	public: \
		test_runner_wrapper() : logiqa::test("", 0, "") {} \
		test_runner_wrapper(int index) : logiqa::test(#name, index, tags) { session().tests_[logiqa_unique_name()] = this; } \
		void logiqa_body() override; \
		const auto& logiqa_param() { return params.get_values()[logiqa_param_num()]; } \
		void logiqa_run() override \
		{ \
			prep_param(static_cast<const void*>(&logiqa_param())); \
			try { setup(); } \
			LOGIQA_EXCEPTION_CATCHER(fixture_setup) \
			test::logiqa_run(); \
			try { teardown(); } \
			LOGIQA_EXCEPTION_CATCHER(fixture_teardown) \
		} \
	}; \
	static std::vector<std::shared_ptr<test_runner_wrapper>> instances() \
	{ \
		std::vector<std::shared_ptr<test_runner_wrapper>> result; \
		for (int p = 0; p < params.get_values().size(); ++p) \
			result.push_back(std::make_shared<test_runner_wrapper>(p)); \
		return result; \
	} \
	static std::vector<std::shared_ptr<test_runner_wrapper>> votcw = instances(); \
} } }\
void logiqa::tests::name::test_runner_wrapper::logiqa_body()


// Test body assertions...
#define TEST_HALT throw std::runtime_error("Halting test.");
#define REPORT_PASS(assert_type, msg) logiqa_report_pass(__FILE__, __LINE__, assert_type, msg);
#define REPORT_FAIL(assert_type, value, expected, tolerance, msg) logiqa_report_fail(__FILE__, __LINE__, assert_type, value, expected, tolerance, msg);

#define ASSERT_PASS REPORT_PASS("ASSERT_PASS", "")
#define ASSERT_FAIL REPORT_FAIL("ASSERT_FAIL", "", "", "", "")
#define ASSERT_EQ(x, y) if (x == y) { REPORT_PASS("ASSERT_EQ", "") } else { REPORT_FAIL("ASSERT_EQ", std::to_string(x), std::to_string(y), "", "") };
#define ASSERT_EQ_STR(x, y) if (x.compare(y) == 0) { REPORT_PASS("ASSERT_EQ_STR", "") } else { REPORT_FAIL("ASSERT_EQ_STR", x, y, "", "") };
#define ASSERT_LEQ(x, y) if (x <= y) { REPORT_PASS("ASSERT_LEQ", "") } else { REPORT_FAIL("ASSERT_LEQ", std::to_string(x), std::to_string(y), "", ""); };
#define ASSERT_GEQ(x, y) if (x >= y) { REPORT_PASS("ASSERT_GEQ", "") } else { REPORT_FAIL("ASSERT_GEQ", std::to_string(x), std::to_string(y), "", ""); };
#define ASSERT_LT(x, y) if (x < y) { REPORT_PASS("ASSERT_LT", "") } else { REPORT_FAIL("ASSERT_LT", std::to_string(x), std::to_string(y), "", ""); };
#define ASSERT_GT(x, y) if (x > y) { REPORT_PASS("ASSERT_GT", "") } else { REPORT_FAIL("ASSERT_GT", std::to_string(x), std::to_string(y), "", ""); };
#define ASSERT_NEAR(x, y, e) if (abs(y-x) <= e) { REPORT_PASS("ASSERT_NEAR", "") } else { REPORT_FAIL("ASSERT_NEAR", std::to_string(x), std::to_string(y), std::to_string(e), ""); };
#define ASSERT(x) ASSERT_EQ(x, true);

// Macro for defining main function, user can perform custom global setup and teardown (applied to entire test program) also processed -jahoutf arguments.
// Must explicitly call RUNALL, RUNALL_RANDOMIZED or explicit tests?
#define LOGIQA_TEST_RUNNER \
void test_runner_main(); \
int main(int argc, char** argv) \
{ \
	if (logiqa::_::test_runner_arguments(argc, argv, logiqa::session())) \
		test_runner_main(); \
	return 0; \
} \
void test_runner_main()

// Run all the tests.... used if user has global startup and teardown functionality in main....
#define RUNALL logiqa::_::test_runner_run_tests(logiqa::session());

// Run all the tests, silence the output...
#define SILENCE logiqa::session().silence_ = true;

// Shuffle the tests...
#define SHUFFLE logiqa::session().shuffle_ = true;

// Add a user reporter...
#define REPORT(user_reporter) logiqa::session().report_.push_back(std::make_shared<user_reporter>());

// Set the user event hooks...
#define EVENT(user_event) logiqa::session().event_ = std::make_unique<user_event>();

// exception catcher macro...
#define LOGIQA_EXCEPTION_CATCHER(location) catch(const std::exception& e) { logiqa_report_exception(__FILE__, __LINE__, "std::exception", #location, e.what()); } \
catch (...) { logiqa_report_exception(__FILE__, __LINE__, "unknown exception", #location, ""); }




// Source code...

// namespace logiqa
namespace logiqa
{

	class test;

	typedef std::vector<test*> test_list;

	namespace results
	{
		struct result
		{
			result(const std::string& filename, unsigned int line_number, const std::string& t, const std::string& msg)
				: type_(t), msg_(msg), filename_(filename), line_number_(line_number) {}

			std::string type_;
			std::string msg_;
			std::string filename_;
			unsigned int line_number_;
		};

		typedef result pass;

		struct fail : public result
		{
			fail(const result& r, const std::string& value, const std::string& expected, const std::string& tolerance)
				: result(r.filename_, r.line_number_, r.type_, r.msg_), value_(value), expected_(expected), tolerance_(tolerance) {}

			std::string value_;
			std::string expected_;
			std::string tolerance_;
		};

		struct exception : public result
		{
			exception(const result& r, const std::string& location)
				: result(r.filename_, r.line_number_, r.type_, r.msg_), location_(location) {}

			std::string location_;
		};

		struct summary
		{
			unsigned int test_passed_ = 0;
			unsigned int test_failed_ = 0;
			unsigned int test_empty_ = 0;
			unsigned int exceptions_ = 0;
			unsigned int assertions_passed_ = 0;
			unsigned int assertions_failed_ = 0;
			unsigned int skipped_ = 0;
			unsigned int duration_ = 0;
		};

	}

	class report_interface
	{
	public:
		virtual void report(const test_list& tests, const results::summary& summary) {}
		virtual std::string name() = 0;
	};

	class event_interface
	{
	public:
		virtual void message(const std::string& msg) {}
		virtual void case_start(const test& test) {}
		virtual void case_success(const test& test, const results::pass& info) {}
		virtual void case_fail(const test& test, const results::fail& info) {}
		virtual void case_exception(const test& test, const results::exception& info) {}
		virtual void case_end(const test& test) {}
		virtual void suite_start(const test_list& tests) {}
		virtual void suite_end(const test_list& tests, const results::summary& summary) {}
		virtual void list_test(const test& test, bool list_tags) {}
		virtual void list_tag(const std::string& tag, unsigned int count) {}
	};

	namespace _
	{
		class instance
		{
		public:
			instance()
			{
				int y = 0;
				++y;
			}

			~instance()
			{
				int y = 0;
				++y;
			}

			std::map<std::string, test*> tests_;
			std::shared_ptr<event_interface> event_;
			std::vector<std::shared_ptr<report_interface>> report_;
			std::vector<std::string> tags_;
			std::string test_runner_name_;
			std::string test_runner_path_;

			bool shuffle_ = false;
			bool list_ = false;
			bool list_tags_ = false;
			bool silence_ = false;

			static instance& get_instance()
			{
				static instance INSTANCE;
				return INSTANCE;
			}
		};

	}

	static _::instance& session()
	{
		return _::instance::get_instance();
	}


	class test
	{
	public:
		test(const std::string& name, unsigned int index, const std::string& tags) : name_(name), param_index_(index), duration_(0), tags_(" " + tags + " ") {}

		// The test body that is invoked.
		virtual void logiqa_body() {}

		// invoke test...
		virtual void logiqa_run()
		{
			// start timer...
			auto begin = std::chrono::steady_clock::now();
			try { logiqa_body(); }
			LOGIQA_EXCEPTION_CATCHER(test_body)
				auto end = std::chrono::steady_clock::now();
			duration_ = static_cast<unsigned int>(std::chrono::duration_cast<std::chrono::milliseconds>(begin - end).count());
		}

		// unique name...
		std::string logiqa_unique_name() const 
		{ 
			return name_ + "[" + std::to_string(param_index_) + "]";
		}

		// report pass...
		void logiqa_report_pass(const std::string& filename, unsigned int line_number, const std::string& assert_type, const std::string& msg)
		{
			passes_.push_back(results::pass(filename, line_number, assert_type, msg));
			logiqa::session().event_->case_success(*this, passes_.back());
		}

		// report fail...
		void logiqa_report_fail(const std::string& filename, unsigned int line_number, const std::string& assert_type, const std::string& value, const std::string& expected, const std::string& tolerance, const std::string& msg)
		{
			fails_.push_back(results::fail(results::result(filename, line_number, assert_type, msg), value, expected, tolerance));
			logiqa::session().event_->case_fail(*this, fails_.back());
		}

		// report exception...
		void logiqa_report_exception(const std::string& filename, unsigned int line_number, const std::string& exception_type, const std::string& location, const std::string& what)
		{
			exceptions_.push_back(results::exception(results::result(filename, line_number, exception_type, what), location));
			logiqa::session().event_->case_exception(*this, exceptions_.back());
		}

		// true if the supplied tag is present in the tags...
		bool logiqa_tagged(const std::string& tag) const
		{
			// first check any of our tags...
			bool valid = tags_.find(std::string(" " + tag + " ")) != std::string::npos;
			
			// check the name...
			if (!valid)
				valid = name_ == tag;

			// check if tag is accessing a specfiic param..
			if (!valid)
				valid = logiqa_unique_name() == tag;
				
			return valid;
		}
		const std::string& logiqa_tags() const { return tags_; }

		// run-time test info...
		const std::string& logiqa_name() const { return name_; }
		unsigned int logiqa_param_num() const { return param_index_; }

		// results...
		const std::vector<results::pass>& logiqa_result_passes() const { return passes_; }
		const std::vector<results::fail>& logiqa_result_fails() const { return fails_; }
		const std::vector<results::exception>& logiqa_result_exceptions() const { return exceptions_; }
		unsigned int logiqa_result_total() const { return static_cast<unsigned int>(passes_.size() + fails_.size()); }
		unsigned int logiqa_result_duration_ms() const { return duration_; }

	private:
		std::string name_;
		std::string tags_;
		unsigned int param_index_;
		unsigned int duration_;
		std::vector<results::exception> exceptions_;
		std::vector<results::pass> passes_;
		std::vector<results::fail> fails_;
	};

	class fixture
	{
	public:
		virtual void setup() {}
		virtual void teardown() {}
	protected:
		virtual void prep_param(const void* param) {}
	};

	template<class T>
	class param
	{
		std::vector<T> values_;
	public:
		param(const std::vector<T>& vs) : values_(vs) {}
		const std::vector<T>& get_values() const { return values_; }
	};

	template<class T>
	class fixture_param : public fixture
	{
		const T* param_value_;
	public:
		const T& logiqa_param()
		{
			if (param_value_)
				return *param_value_;
			throw std::runtime_error("Unable to retrieve param.");
		}
	protected:
		void prep_param(const void* param_value)
		{
			param_value_ = reinterpret_cast<const T*>(param_value);
		}
	};


	namespace results
	{
		static summary summarise(const test_list& tests, const std::map<std::string, test*>& all_tests)
		{
			summary total;
			for (auto t = 0; t < tests.size(); ++t)
			{
				total.assertions_passed_ += static_cast<unsigned int>(tests[t]->logiqa_result_passes().size());
				total.assertions_failed_ += static_cast<unsigned int>(tests[t]->logiqa_result_fails().size());
				total.exceptions_ += static_cast<unsigned int>(tests[t]->logiqa_result_exceptions().size());
				total.duration_ += tests[t]->logiqa_result_duration_ms();

				if (!tests[t]->logiqa_result_fails().empty())
					++total.test_failed_;
				else if (!tests[t]->logiqa_result_passes().empty())
					++total.test_passed_;
				else
					++total.test_empty_;
			}
			total.skipped_ = static_cast<unsigned int>(all_tests.size() - tests.size());
			return total;
		}

	}

}





#ifdef LOGIQA_INCLUDE_DEFAULT_CONSOLE
#include <iostream>
namespace logiqa
{
	// stdout output...
	class console : public event_interface
	{
	public:

#ifdef LOGIQA_CONSOLE_COLOUR_DISABLED
		void black(const std::string& msg) { std::cout << msg; }
		void red(const std::string& msg) { std::cout << msg; }
		void green(const std::string& msg) { std::cout << msg; }
		void yellow(const std::string& msg) { std::cout << msg; }
		void blue(const std::string& msg) { std::cout << msg; }
		void magenta(const std::string& msg) { std::cout << msg; }
		void cyan(const std::string& msg) { std::cout << msg; }
		void white(const std::string& msg) { std::cout << msg; }
		void inverse(const std::string& msg) { std::cout << msg; }
#elif LOGIQA_ANSI
		void black(const std::string& msg) { std::cout << std::string("\033[1;30m" + msg + "\033[0m"); }
		void red(const std::string& msg) { std::cout << std::string("\033[1;31m" + msg + "\033[0m"); }
		void green(const std::string& msg) { std::cout << std::string("\033[1;32m" + msg + "\033[0m"); }
		void yellow(const std::string& msg) { std::cout << std::string("\033[1;33m" + msg + "\033[0m"); }
		void blue(const std::string& msg) { std::cout << std::string("\033[1;34m" + msg + "\033[0m"); }
		void magenta(const std::string& msg) { std::cout << std::string("\033[1;35m" + msg + "\033[0m"); }
		void cyan(const std::string& msg) { std::cout << std::string("\033[1;36m" + msg + "\033[0m"); }
		void white(const std::string& msg) { std::cout << std::string("\033[0;37m" + msg + "\033[0m"); }
		void inverse(const std::string& msg) { std::cout << std::string("\033[1;7m" + msg + "\033[0m"); }
#elif LOGIQA_WIN
		class colour
		{
		public:
			colour(const std::string& msg, int c)
			{
				HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
				CONSOLE_SCREEN_BUFFER_INFO cbInfo;
				GetConsoleScreenBufferInfo(hConsole, &cbInfo);
				restore_ = cbInfo.wAttributes;
				SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), c);
				std::cout << msg;
			}
			~colour()
			{
				SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), restore_);
			}
			int restore_;
		};
		void black(const std::string& msg) { colour(msg, 16); }
		void red(const std::string& msg) { colour(msg, 12); }
		void green(const std::string& msg) { colour(msg, 10); }
		void yellow(const std::string& msg) { colour(msg, 14); }
		void blue(const std::string& msg) { colour(msg, 9); }
		void magenta(const std::string& msg) { colour(msg, 13); }
		void cyan(const std::string& msg) { colour(msg, 11); }
		void white(const std::string& msg) { colour(msg, 15); }
		void inverse(const std::string& msg) { colour(msg, 10); }
#endif
		virtual void message(const std::string& msg) { std::cout << msg; }
		std::string header_string(const test& test)
		{
			return test.logiqa_param_num() ? test.logiqa_unique_name() : test.logiqa_name();
		}
		virtual void header(const test& test) 
		{ 
			message("\n : "); message(test.logiqa_name());
			if (test.logiqa_param_num())
			{
				message("["); cyan(std::to_string(test.logiqa_param_num())); message("]");
			}
			message(" : ");
		}
		virtual void line(const results::result& r)
		{
			cyan(r.filename_ + ":"); yellow(std::to_string(r.line_number_));
		}
		virtual void duration(unsigned int d) 
		{ 
			//message("(" + std::to_string(d) + "ms)"); 
		}
		
		void results_bar(unsigned int successes, unsigned int failures)
		{
			if (!successes && !failures)
				return;

			unsigned int total = successes + failures, length = 20;
			float percentage = static_cast<float>(failures) / static_cast<float>(total);
			unsigned int f_bar = static_cast<int>(percentage * static_cast<float>(length));
			if (!f_bar && failures) { f_bar = 1; }
			unsigned int s_bar = length - f_bar;
			message("|"); green(std::string(s_bar, '=')); red(std::string(f_bar, '=')); message("|");
		}
		virtual void case_success(const test& test, const results::pass& result)
		{
			if (!result.msg_.empty()) { header(test); message(" "); line(result); message(" " + result.msg_); }
		}
		virtual void case_fail(const test& test, const results::fail& result)
		{
			if (test.logiqa_result_fails().size() == 1 && test.logiqa_result_exceptions().empty())
				message("\n\n  " + header_string(test) + " : ");

			message("\n\t"); line(result); message(" ");
			if (result.value_.empty() && result.expected_.empty() && result.tolerance_.empty())
				message(result.type_ + "()");
			if (!result.value_.empty() && !result.expected_.empty() && result.tolerance_.empty())
				message(result.type_ + "(" + result.value_ + ", " + result.expected_ + ")");
			if (!result.value_.empty() && !result.expected_.empty() && !result.tolerance_.empty())
				message(result.type_ + "(" + result.value_ + ", " + result.expected_ + ") tolerance " + result.tolerance_);
			if (!result.msg_.empty())
				message("\n : " + result.msg_);
			
		}
		virtual void case_exception(const test& test, const results::exception& result)
		{
			if (test.logiqa_result_exceptions().size() == 1 && test.logiqa_result_fails().empty())
				message("\n\n  " + header_string(test) + " : ");

			message("\n\t"); line(result); message(" "); magenta("{"); yellow(result.location_); magenta("}");
			if (!result.msg_.empty())
				magenta(" " + result.msg_);
			
		}
		virtual void case_end(const test& test)
		{
			if (!test.logiqa_result_exceptions().empty())
			{
				message("\n! "); magenta(header_string(test)); message(" : "); message(std::to_string(test.logiqa_result_exceptions().size()) + " exceptions occured. \n");
			}
			else if (test.logiqa_result_fails().empty() && !test.logiqa_result_passes().empty())
			{
				message("\n. "); green(header_string(test)); message(" : "); message(std::to_string(test.logiqa_result_passes().size()) + "/" + std::to_string(test.logiqa_result_total()) + " assertions passed. "); duration(test.logiqa_result_duration_ms());
			}
			else if (!test.logiqa_result_fails().empty())
			{
				message("\nx "); red(header_string(test)); message(" : "); message(std::to_string(test.logiqa_result_fails().size()) + "/" + std::to_string(test.logiqa_result_total()) + " assertions failed.");
			}
		}
		virtual void suite_start(const test_list& tests_to_run)
		{
			message("Running " + std::to_string(tests_to_run.size()) + " tests. (type \"?\" for help) ");
			//if (session().shuffle_)
			if (session().shuffle_)
				cyan(" [Shuffle] ");
			unsigned int skipped = static_cast<unsigned int>(session().tests_.size() - tests_to_run.size());
			if (skipped)
				yellow(" [" + std::to_string(skipped) + " tests are disabled.]");
			message("\n|====================|\n");
		}

		virtual void suite_end(const test_list& tests_to_run, const results::summary& summary)
		{
			message("\n\n");
			unsigned int total = summary.test_passed_ + summary.test_failed_ + summary.exceptions_;
			unsigned int total_assertions = summary.assertions_failed_ + summary.assertions_passed_;

			if (summary.test_passed_)
			{
				results_bar(summary.test_passed_, summary.test_failed_); message("\n");
				message("     Passed | " + std::to_string(summary.test_passed_) + " (" + std::to_string(summary.assertions_passed_) + "/" + std::to_string(total_assertions) + " assertions)\n");
			}
			if (summary.test_failed_)
				message("     Failed | " + std::to_string(summary.test_failed_) + " (" + std::to_string(summary.assertions_failed_) + "/" + std::to_string(total_assertions) + " assertions)\n");
			if (summary.exceptions_)
				message(" Exceptions | " + std::to_string(summary.exceptions_) + "\n");
			
			if (!total)
				message("  No Tests run.\n|====================|\n");
			message("      Total | " + std::to_string(total) + " "); duration(summary.duration_); message("\n"); 
			if (summary.test_empty_)
				message("\n" + std::to_string(summary.test_empty_) + " test(s) empty.");
			if (summary.skipped_)
				yellow("\n " + std::to_string(summary.skipped_) + " test(s) skipped.");
			message("\n");
		}

		void list_test(const test& test, bool list_tags)
		{
			if (!test.logiqa_param_num())
			{
				message(test.logiqa_name());
				if (!test.logiqa_tags().empty() && list_tags)
				{
					message(" ["); cyan(test.logiqa_tags()); message("]\n");
				}
				else
					message("\n");
			}
		}

		void list_tag(const std::string& tag, unsigned int count)
		{
			cyan(tag + "\n");
		}

		
	};
}
#endif


// xUnit output...
#ifdef LOGIQA_INCLUDE_DEFAULT_XUNIT
#include <fstream>
#include <sstream>
namespace logiqa
{
	class xUnit : public report_interface
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
			testsuite_raii(std::ostringstream& oss, const std::string& name, const results::summary& summary, const std::string& indent)
				: oss_(oss)
			{
				oss_ << "\n" << indent << "<testsuite name=\"" << name << "\" tests=\"" << (summary.test_passed_ + summary.test_failed_ + summary.exceptions_) << "\" failures=\"" << summary.test_passed_ << "\" disabled=\"" << summary.skipped_ << "\" errors=\"" << summary.exceptions_ << "\" time=\"" << summary.duration_ << "\">";
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
			testcase_raii(std::ostringstream& oss, const test& test, const std::string& indent)
				: oss_(oss)
			{
				oss_ << "\n" << indent << "<testcase name=\"" << test.logiqa_unique_name() << "\" status=\"run\" time=\"" << test.logiqa_result_duration_ms() << ">";
				for (auto f = test.logiqa_result_fails().begin(); f != test.logiqa_result_fails().end(); ++f)
					oss_ << "\n" << indent << "  " << "<failure message=\"" << xUnit::escape_xml_chars(f->msg_) << "\"></failure>";
				for (auto f = test.logiqa_result_fails().begin(); f != test.logiqa_result_fails().end(); ++f)
					oss_ << "\n" << indent << "  " << "<failure message=\"" << xUnit::escape_xml_chars(f->msg_) << "\"></failure>";
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

		void report(const test_list& tests, const results::summary& summary)
		{
			std::ostringstream oss;
			oss << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
			{
				testsuite_raii ts(oss, session().test_runner_name_, summary, "");
				for (unsigned int t = 0; t < tests.size(); ++t)
					testcase_raii tc(oss, *tests[t], "  ");
			}
			std::ofstream file(filepath_);
			if (file)
				file << oss.str();
			else
				throw std::runtime_error("Unable to open file " + filepath_);
		}
	};
}
#endif


namespace logiqa
{
	namespace _
	{
	
		static void test_runner_run_tests(_::instance& session)
		{
			//auto inst = session();

			// assert there is an event or at least a stub in the event...
			if (session.silence_)
				session.event_.reset(new event_interface());
			
			// get all the tests to run...
			test_list tests_to_run;
			tests_to_run.reserve(session.tests_.size());
			if (!session.tags_.empty())
			{
				for (auto itr = session.tests_.begin(); itr != session.tests_.end(); ++itr)
				{
					bool is_tagged = false;
					for (unsigned int t = 0; t < session.tags_.size() && !is_tagged; ++t)
						is_tagged = itr->second->logiqa_tagged(session.tags_[t]);
					if (is_tagged)
						tests_to_run.push_back(itr->second);
				}
			}
			else
			{
				for (auto itr = session.tests_.begin(); itr != session.tests_.end(); ++itr)
					tests_to_run.push_back(itr->second);
			}

			// if we are listing them...
			if (session.list_ || session.list_tags_)
			{
				// If we are listing just the tags...
				if (!session.list_ && session.list_tags_)
				{
					std::string buf;
					std::map<std::string, unsigned int> tags;
					for (unsigned int t = 0; t < tests_to_run.size(); ++t)
					{
						// tokenise the tags...
						std::stringstream ss(tests_to_run[t]->logiqa_tags());
						while (ss >> buf)
							++tags[buf];
					}

					// output the tags...
					for (auto itr = tags.begin(); itr != tags.end(); ++itr)
						session.event_->list_tag(itr->first, itr->second);
				}
				else
				{
					for (unsigned int t = 0; t < tests_to_run.size(); ++t)
						session.event_->list_test(*tests_to_run[t], session.list_tags_);
				}
				return;
			}

			// if shuffle, randomise...
			if (session.shuffle_)
				std::random_shuffle(tests_to_run.begin(), tests_to_run.end());
			
			// run the tests...
			logiqa::session().event_->suite_start(tests_to_run);
			for (unsigned int t = 0; t < tests_to_run.size(); ++t)
			{
				logiqa::session().event_->case_start(*tests_to_run[t]);
				tests_to_run[t]->logiqa_run();
				logiqa::session().event_->case_end(*tests_to_run[t]);
			}
			results::summary total = results::summarise(tests_to_run, session.tests_);
			logiqa::session().event_->suite_end(tests_to_run, total);

			// report
			for (auto reporter = session.report_.begin(); reporter != session.report_.end(); ++reporter)
			{
				try { (*reporter)->report(tests_to_run, total); }
				catch (const std::exception& e) { std::cerr << "Exception thrown in reporter " << (*reporter)->name() << ", " << e.what() << "\n"; }
				catch (...) { std::cerr << "Unknown exception thrown in reporter " << (*reporter)->name() << "\n"; }
			}
		}

		static bool test_runner_arguments(int argc, char** argv, _::instance& session)
		{
			// by default, use console output...
#ifdef LOGIQA_INCLUDE_DEFAULT_CONSOLE
			session.event_.reset(new console());
#endif
			session.test_runner_name_ = std::string(*argv);
#ifdef LOGIQA_WIN
			auto itr = session.test_runner_name_.rfind('\\');
#endif
#ifdef LOGIQA_NIX	
			auto itr = session.test_runner_name_.rfind('/');
#endif
			if (itr != 0 && itr != std::string::npos)
			{
				session.test_runner_path_ = session.test_runner_name_.substr(0, itr + 1); 
				session.test_runner_name_ = session.test_runner_name_.substr(itr+1);
			}
			else
				session.test_runner_name_ = "";
				

			for (int i = 1; i < argc; ++i)
			{
				std::string arg(*(argv + i));
				if (arg == "-silent")
					logiqa::session().silence_ = true;
				else if (arg == "-tags")
					logiqa::session().list_tags_ = true;
				else if (arg == "-list")
					logiqa::session().list_ = true;
				else if (arg == "-shuffle")
					logiqa::session().shuffle_ = true;
#ifdef LOGIQA_INCLUDE_DEFAULT_XUNIT
				else if (arg.find("-xunit=") != std::string::npos)
				{
					auto xunit = std::make_shared<xUnit>(arg.substr(7));
					session.report_.push_back(xunit);
				}
#endif
				else if (arg == "?" || arg == "-help")
				{
					std::string str;
					str.append("LogiQA test runner.\n");
					str.append("Usage: > " + session.test_runner_name_ + " [-silent] [-list] [-shuffle] [-xunit=\"filename.xml\"] [?] test1 test2 ...\n");
					str.append("\n");
					str.append("-shuffle : shuffle the tests before running them.\n");
					str.append("-silent  : silence events. No console output.\n"); 
#ifdef LOGIQA_INCLUDE_DEFAULT_XUNIT
					str.append("-xunit   : specify an xml filename to report xUnit to.\n");
#endif
					str.append("-list    : list the tests in this test program.\n");
					str.append("-tags    : list the tags.\n");
					str.append("\n");
					str.append("Using -list and -tags together, will list the tests and associated tags for each test.\n");
					str.append("Pattern matching works on tags, name, and unique name (i.e \"testname[42]\") will match testname with param index 42.\n");
					str.append("All tests except those matching patterns will be disabled/skipped.\n");
					
					session.event_->message(str);
					return false;
				}
				else
					logiqa::session().tags_.push_back(arg);
			}
			return true;
		}
	}
}




#endif // LOGIQA_HPP
