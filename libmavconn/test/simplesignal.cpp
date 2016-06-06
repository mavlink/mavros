/**
 * Test Simple Signal library.
 *
 * This test is stripped out from header file.
 *
 * Compile command:
 *     g++ -Wall -O2 -std=c++11 -pthread -Iinclude test/simplesignal.cpp -lrt -o /tmp/signaltest
 */

#include <string>
#include <stdarg.h>
#include <mavconn/simplesignal.h>


static std::string string_printf (const char *format, ...) __attribute__ ((__format__ (__printf__, 1, 2)));
static std::string
string_printf (const char *format, ...)
{
  std::string result;
  char *str = NULL;
  va_list args;
  va_start (args, format);
  if (vasprintf (&str, format, args) >= 0)
    result = str;
  va_end (args);
  if (str)
    free (str);
  return result;
}

static uint64_t
timestamp_benchmark ()
{
  struct timespec tp = { 0, 0 };
  if (__builtin_expect (clock_gettime (CLOCK_MONOTONIC, &tp) < 0, 0))
    {
      perror ("failed in clock_gettime");
      exit (-1);
    }
  uint64_t stamp = tp.tv_sec * 1000000000ULL + tp.tv_nsec;
  return stamp;
}

struct TestCounter {
  static uint64_t get     ();
  static void     set     (uint64_t);
  static void     add2    (void*, uint64_t);
};

namespace { // Anon
void        (*test_counter_add2) (void*, uint64_t) = TestCounter::add2; // external symbol to prevent easy inlining
static uint64_t test_counter_var = 0;
} // Anon

class BasicSignalTests {
  static std::string accu;
  struct Foo {
    char
    foo_bool (float f, int i, std::string s)
    {
      accu += string_printf ("Foo: %.2f\n", f + i + s.size());
      return true;
    }
  };
  static char
  float_callback (float f, int, std::string)
  {
    accu += string_printf ("float: %.2f\n", f);
    return 0;
  }
public:
  static void
  run()
  {
    accu = "";
    mavconn::signal::Signal<char (float, int, std::string)> sig1;
    size_t id1 = sig1 += float_callback;
    size_t id2 = sig1 += [] (float, int i, std::string) { accu += string_printf ("int: %d\n", i); return 0; };
    size_t id3 = sig1 += [] (float, int, const std::string &s) { accu += string_printf ("string: %s\n", s.c_str()); return 0; };
    sig1.emit (.3, 4, "huhu");
    bool success;
    success = sig1 -= id1; assert (success == true);  success = sig1 -= id1; assert (success == false);
    success = sig1 -= id2; assert (success == true);  success = sig1 -= id3; assert (success == true);
    success = sig1 -= id3; assert (success == false); success = sig1 -= id2; assert (success == false);
    Foo foo;
    sig1 += mavconn::signal::slot (foo, &Foo::foo_bool);
    sig1 += mavconn::signal::slot (&foo, &Foo::foo_bool);
    sig1.emit (.5, 1, "12");

    mavconn::signal::Signal<void (std::string, int)> sig2;
    sig2 += [] (std::string msg, int) { accu += string_printf ("msg: %s", msg.c_str()); };
    sig2 += [] (std::string, int d)   { accu += string_printf (" *%d*\n", d); };
    sig2.emit ("in sig2", 17);

    accu += "DONE";

    const char *expected =
      "float: 0.30\n"
      "int: 4\n"
      "string: huhu\n"
      "Foo: 3.50\n"
      "Foo: 3.50\n"
      "msg: in sig2 *17*\n"
      "DONE";
    assert (accu == expected);
  }
};
std::string BasicSignalTests::accu;


class TestCollectorVector {
  static int handler1   ()  { return 1; }
  static int handler42  ()  { return 42; }
  static int handler777 ()  { return 777; }
  public:
  static void
  run ()
  {
    mavconn::signal::Signal<int (), mavconn::signal::CollectorVector<int>> sig_vector;
    sig_vector += handler777;
    sig_vector += handler42;
    sig_vector += handler1;
    sig_vector += handler42;
    sig_vector += handler777;
    std::vector<int> results = sig_vector.emit();
    const std::vector<int> reference = { 777, 42, 1, 42, 777, };
    assert (results == reference);
  }
};

class TestCollectorUntil0 {
  bool check1, check2;
  TestCollectorUntil0() : check1 (0), check2 (0) {}
  bool handler_true  ()  { check1 = true; return true; }
  bool handler_false ()  { check2 = true; return false; }
  bool handler_abort ()  { abort(); }
  public:
  static void
  run ()
  {
    TestCollectorUntil0 self;
    mavconn::signal::Signal<bool (), mavconn::signal::CollectorUntil0<bool>> sig_until0;
    sig_until0 += mavconn::signal::slot (self, &TestCollectorUntil0::handler_true);
    sig_until0 += mavconn::signal::slot (self, &TestCollectorUntil0::handler_false);
    sig_until0 += mavconn::signal::slot (self, &TestCollectorUntil0::handler_abort);
    assert (!self.check1 && !self.check2);
    const bool result = sig_until0.emit();
    assert (!result && self.check1 && self.check2);
  }
};

class TestCollectorWhile0 {
  bool check1, check2;
  TestCollectorWhile0() : check1 (0), check2 (0) {}
  bool handler_0     ()  { check1 = true; return false; }
  bool handler_1     ()  { check2 = true; return true; }
  bool handler_abort ()  { abort(); }
  public:
  static void
  run ()
  {
    TestCollectorWhile0 self;
    mavconn::signal::Signal<bool (), mavconn::signal::CollectorWhile0<bool>> sig_while0;
    sig_while0 += mavconn::signal::slot (self, &TestCollectorWhile0::handler_0);
    sig_while0 += mavconn::signal::slot (self, &TestCollectorWhile0::handler_1);
    sig_while0 += mavconn::signal::slot (self, &TestCollectorWhile0::handler_abort);
    assert (!self.check1 && !self.check2);
    const bool result = sig_while0.emit();
    assert (result == true && self.check1 && self.check2);
  }
};

static void
bench_simple_signal()
{
  mavconn::signal::Signal<void (void*, uint64_t)> sig_increment;
  sig_increment += test_counter_add2;
  const uint64_t start_counter = TestCounter::get();
  const uint64_t benchstart = timestamp_benchmark();
  uint64_t i;
  for (i = 0; i < 999999; i++)
    {
      sig_increment.emit (NULL, 1);
    }
  const uint64_t benchdone = timestamp_benchmark();
  const uint64_t end_counter = TestCounter::get();
  assert (end_counter - start_counter == i);
  printf ("OK\n  Benchmark: mavconn::signal::Signal: %fns per emission (size=%zu): ", size_t (benchdone - benchstart) * 1.0 / size_t (i),
          sizeof (sig_increment));
}

static void
bench_callback_loop()
{
  void (*counter_increment) (void*, uint64_t) = test_counter_add2;
  const uint64_t start_counter = TestCounter::get();
  const uint64_t benchstart = timestamp_benchmark();
  uint64_t i;
  for (i = 0; i < 999999; i++)
    {
      counter_increment (NULL, 1);
    }
  const uint64_t benchdone = timestamp_benchmark();
  const uint64_t end_counter = TestCounter::get();
  assert (end_counter - start_counter == i);
  printf ("OK\n  Benchmark: callback loop: %fns per round: ", size_t (benchdone - benchstart) * 1.0 / size_t (i));
}

uint64_t
TestCounter::get ()
{
  return test_counter_var;
}

void
TestCounter::set (uint64_t v)
{
  test_counter_var = v;
}

void
TestCounter::add2 (void*, uint64_t v)
{
  test_counter_var += v;
}

int
main (int   argc,
      char *argv[])
{
  printf ("Signal/Basic Tests: ");
  BasicSignalTests::run();
  printf ("OK\n");

  printf ("Signal/CollectorVector: ");
  TestCollectorVector::run();
  printf ("OK\n");

  printf ("Signal/CollectorUntil0: ");
  TestCollectorUntil0::run();
  printf ("OK\n");

  printf ("Signal/CollectorWhile0: ");
  TestCollectorWhile0::run();
  printf ("OK\n");

  printf ("Signal/Benchmark: mavconn::signal::Signal: ");
  bench_simple_signal();
  printf ("OK\n");

  printf ("Signal/Benchmark: callback loop: ");
  bench_callback_loop();
  printf ("OK\n");

  return 0;
}


