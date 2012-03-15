
#include <boost/interprocess/managed_shared_memory.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <bitset>

using namespace boost::interprocess;

typedef double value_t;

std::string dataStrY;
std::string dataStrU;
std::string dataStrV;

int main(int argc,char **argv)
{
	const char *name = "vcmImage182darwin";
	managed_shared_memory *shm = new managed_shared_memory(open_only,name);
	
	typedef managed_shared_memory::const_named_iterator const_named_it;
	const_named_it named_beg = shm->named_begin();
	const_named_it named_end = shm->named_end();

	const char *curkey = named_beg->name();
	bool find = false;
	do 
	{
		if (find) 
		{	
			const char *key = named_beg->name();
			std::pair<value_t*, std::size_t> ret;
			ret = shm->find<value_t>(key);
			value_t *pr = ret.first;
			int n = ret.second;
			if (pr != NULL) 
			{
//				for (int i=0; i < n; i++) {
//					std::cout << pr[i] << ' ';
//				}	
				std::cout << n << std::endl;
				if (std::string(key) == std::string("labelB"))
				{
					std::cout << "yuyv data" << std::endl;
					for (int i = 0; i < n; i++)
					{
						uint64_t iptr = (uint64_t)pr[i]; 
						uint32_t first  = (iptr & 0xFFFFFFFF00000000) >> 32;
						uint32_t second = (iptr & 0xFFFFFFFF);
						std::cout << std::bitset<64>(pr[i])<< std::endl;
//						std::cout << first << ' ' << second << ' ';
					}
//					std::cout << std::endl;
				}
			}
			std::cout << key << std::endl;
		}
		else
		{
			const managed_shared_memory::char_type *name = named_beg->name();
			std::size_t	name_len = named_beg->name_length();
			if (std::string(curkey) == std::string(name)) 
			{
				find = true;
			}
		}
	} while (++named_beg != named_end);
	return 0;
}
