#pragma once

#include <string>
#include <ostream>
#include <iostream>
#include <chrono>

template <class CharT, class Traits, class Rep, class Period>
std::basic_ostream<CharT, Traits>&
    operator<<(std::basic_ostream<CharT, Traits>& os,
               const std::chrono::duration<Rep, Period>& d);


template<class Clock = std::chrono::high_resolution_clock>
class Timer
{
public:
	enum class TimerState
	{
		Running,
		Stopped
	};

	typedef std::chrono::time_point<Clock> TimePoint;

	Timer(bool autoStart = true) : state(TimerState::Stopped)
	{
		if (autoStart)
		{
			start();
		}
	}

	inline void start()
	{
		if (state == TimerState::Stopped)
		{
			begin = Clock::now();
			state = TimerState::Running;
		}
	}

	inline void stop()
	{
		if (state == TimerState::Running)
		{
			end = Clock::now();
			state = TimerState::Stopped;
		}
	}

	template<class Duration = std::chrono::milliseconds>
	inline Duration get() const
	{
		if (state == TimerState::Stopped)
		{
			return std::chrono::duration_cast<Duration>(end - begin);
		}
		else
		{
			return Duration::zero();
		}
	}

	template<class Duration = std::chrono::milliseconds>
	inline typename Duration::rep count() const
	{
		return get().count();
	}


	template<class CharT>
	inline void report(const CharT* msg = nullptr,
					   std::basic_ostream<CharT>& os = std::cout) const
	{
		if (state == TimerState::Stopped)
		{
			if (msg != nullptr)
			{
				os << msg;
			}
			os << get();
		}
	}

	template<class CharT>
	inline void stopAndReport(const CharT* msg = nullptr,
					   std::basic_ostream<CharT>& os = std::cout)
	{
		stop();
		report(msg, os);
	}


private:
	TimerState state;

	TimePoint begin;
	TimePoint end;
};


#if __cplusplus <= 201703L // C++20 already supported.


// compile time string literal for C++11/14/17
template<class CharT, CharT... chars>
struct StringLiteral
{
	static constexpr size_t len = sizeof...(chars);
	static constexpr CharT const str[sizeof...(chars) + 1] = { chars..., CharT{} };


	static constexpr const CharT* const data()
	{
		return str;
	}

	static constexpr size_t size()
	{
		return len;
	}


};

template<class CharT, CharT... chars>
constexpr size_t StringLiteral<CharT, chars...>::len;

template<class CharT, CharT... chars>
constexpr CharT const StringLiteral<CharT, chars...>::str[sizeof...(chars) + 1];


template<class T, class CharT>
struct TimeDurationStr : StringLiteral<CharT> {};

template<class CharT>
struct TimeDurationStr<std::chrono::nanoseconds, CharT> : StringLiteral<CharT, 'n', 's'> {};

template<class CharT>
struct TimeDurationStr<std::chrono::microseconds, CharT> : StringLiteral<CharT, 'u', 's'> {};

template<class CharT>
struct TimeDurationStr<std::chrono::milliseconds, CharT> : StringLiteral<CharT, 'm', 's'> {};

template<class CharT>
struct TimeDurationStr<std::chrono::seconds, CharT> : StringLiteral<CharT, 's'> {};

template<class CharT>
struct TimeDurationStr<std::chrono::minutes, CharT> : StringLiteral<CharT, 'm'> {};

template<class CharT>
struct TimeDurationStr<std::chrono::hours, CharT> : StringLiteral<CharT, 'h'> {};


template <class CharT, class Traits, class Rep, class Period>
std::basic_ostream<CharT, Traits>&
    operator<<(std::basic_ostream<CharT, Traits>& os,
               const std::chrono::duration<Rep, Period>& d)
{

	os << d.count() << TimeDurationStr<std::chrono::duration<Rep, Period>, CharT>::str;
	return os;
}

#endif
