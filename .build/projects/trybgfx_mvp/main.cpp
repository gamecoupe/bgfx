#include "Win32Application.h"
#include "DXSample.h"

#define WNDW_WIDTH 1600
#define WNDW_HEIHT 900

int main()
{
	DXSample sample(WNDW_WIDTH, WNDW_HEIHT);
	return Win32Application::Run(&sample);
}
