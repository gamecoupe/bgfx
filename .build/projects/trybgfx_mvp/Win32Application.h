#ifndef TRYBGFX_WIN32APP_H_HEADER_GUARD
#define TRYBGFX_WIN32APP_H_HEADER_GUARD

#include "DXSample.h"
#include <Windows.h>

class Win32Application
{
public:
    static int Run(DXSample* pSample);
    static HWND GetHwnd() { return m_hwnd; }

protected:
    static LRESULT CALLBACK WindowProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);

private:
    static HWND m_hwnd;
};

#endif // !TRYBGFX_WIN32APP_H_HEADER_GUARD
