#pragma once

template<class Dervied>
class Singleton
{
public:

	static Dervied& getInstance()
	{
		static Dervied instance;
		return instance;
	}

	Singleton(const Singleton&) = delete;
	Singleton& operator=(const Singleton&) = delete;

protected:
	Singleton() {}
	~Singleton() {}
};
