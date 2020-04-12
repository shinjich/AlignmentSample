#ifndef STRICT
#define STRICT	// 厳密なコードを型を要求する
#endif
#include <windows.h>
#include <tchar.h>
#include <stdio.h>
#include <k4a/k4a.h>

#pragma comment( lib, "k4a.lib" )

#define ENABLE_CSV_OUTPUT		1			// 1=CSV 出力を有効にする

// イメージの解像度
#define COLOR_RESOLUTION_WIDTH	(1280)
#define COLOR_RESOLUTION_HEIGHT	(720)
#define DEPTH_RESOLUTION_WIDTH	(640)
#define DEPTH_RESOLUTION_HEIGHT	(576)

// Win32 アプリ用のパラメータ
static const TCHAR szClassName[] = TEXT("アライメントサンプル");
HWND g_hWnd = NULL;							// アプリケーションのウィンドウ
HBITMAP g_hBMP[4] = { NULL, }, g_hBMPold[4] = { NULL, };	// 表示するビットマップのハンドル
HDC g_hDCBMP[4] = { NULL, };				// 表示するビットマップのコンテキスト
BITMAPINFO g_biBMP[4] = { { 0, }, };		// ビットマップの情報 (解像度やフォーマット)
LPDWORD g_pdwPixel[4] = { NULL, };			// ビットマップの中身の先頭 (ピクセル情報)

// Azure Kinect 用のパラメータ
k4a_device_t g_hAzureKinect = nullptr;		// Azure Kinect のデバイスハンドル
k4a_transformation_t g_hTransformation = nullptr;
k4a_image_t g_hTransformedDepthImage = nullptr;
k4a_image_t g_hTransformedColorImage = nullptr;

// Kinect を初期化する
k4a_result_t CreateKinect()
{
	k4a_result_t hr;

	// Azure Kinect を初期化する
	hr = k4a_device_open( K4A_DEVICE_DEFAULT, &g_hAzureKinect );
	if ( hr == K4A_RESULT_SUCCEEDED )
	{
		// Azure Kinect のカメラ設定
		k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
		config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
		config.color_resolution = K4A_COLOR_RESOLUTION_720P;
		config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
		config.camera_fps = K4A_FRAMES_PER_SECOND_30;
		config.synchronized_images_only = true;
		config.depth_delay_off_color_usec = 0;
		config.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;
		config.subordinate_delay_off_master_usec = 0;
		config.disable_streaming_indicator = false;

		// キャリブレーション情報を取得する
		k4a_calibration_t hCalibration;
		hr = k4a_device_get_calibration( g_hAzureKinect, config.depth_mode, config.color_resolution, &hCalibration );
		if ( hr == K4A_RESULT_SUCCEEDED )
		{
			// 変換イメージを作成する
			g_hTransformation = k4a_transformation_create( &hCalibration );
			hr = k4a_image_create( K4A_IMAGE_FORMAT_DEPTH16, COLOR_RESOLUTION_WIDTH, COLOR_RESOLUTION_HEIGHT, COLOR_RESOLUTION_WIDTH * sizeof(uint16_t), &g_hTransformedDepthImage );
			if ( hr == K4A_RESULT_SUCCEEDED )
			{
				hr = k4a_image_create( K4A_IMAGE_FORMAT_COLOR_BGRA32, DEPTH_RESOLUTION_WIDTH, DEPTH_RESOLUTION_HEIGHT, DEPTH_RESOLUTION_WIDTH * sizeof(uint32_t), &g_hTransformedColorImage );
				if ( hr == K4A_RESULT_SUCCEEDED )
				{
					// Azure Kinect の使用を開始する
					hr = k4a_device_start_cameras( g_hAzureKinect, &config );
					if ( hr == K4A_RESULT_SUCCEEDED )
					{
						return hr;
					}
				}
			}
		}

		// Azure Kinect の使用をやめる
		MessageBox( NULL, TEXT("Azure Kinect が開始できませんでした"), TEXT("エラー"), MB_OK );
		k4a_device_close( g_hAzureKinect );
	}
	else
	{
		MessageBox( NULL, TEXT("Azure Kinect の初期化に失敗 - カメラの状態を確認してください"), TEXT("エラー"), MB_OK );
	}
	return hr;
}

// Kinect を終了する
void DestroyKinect()
{
	if ( g_hAzureKinect )
	{
		// Azure Kinect を停止する
		k4a_device_stop_cameras( g_hAzureKinect );

		// 変換イメージを解放する
		if ( g_hTransformedDepthImage )
		{
			k4a_image_release( g_hTransformedDepthImage );
			g_hTransformedDepthImage = nullptr;
		}
		if ( g_hTransformedColorImage )
		{
			k4a_image_release( g_hTransformedColorImage );
			g_hTransformedColorImage = nullptr;
		}
		if ( g_hTransformation )
		{
			k4a_transformation_destroy( g_hTransformation );
			g_hTransformation = nullptr;
		}

		// Azure Kinect の使用をやめる
		k4a_device_close( g_hAzureKinect );
		g_hAzureKinect = nullptr;
	}
}

// Kinect のメインループ処理
uint32_t KinectProc()
{
	k4a_wait_result_t hr;
	uint32_t uImageSize = 0;

	// キャプチャーする
	k4a_capture_t hCapture = nullptr;
	hr = k4a_device_get_capture( g_hAzureKinect, &hCapture, K4A_WAIT_INFINITE );
	if ( hr == K4A_WAIT_RESULT_SUCCEEDED )
	{
		k4a_image_t hDepthImage, hColorImage;

		// 深度イメージを取得する
		hDepthImage = k4a_capture_get_depth_image( hCapture );

		// カラーイメージを取得する
		hColorImage = k4a_capture_get_color_image( hCapture );

		// カラーから深度に変換する
		const k4a_result_t rTransColorToDepth = k4a_transformation_color_image_to_depth_camera( g_hTransformation, hDepthImage, hColorImage, g_hTransformedColorImage );

		// 深度からカラーに変換する
		const k4a_result_t rTransDepthToColor = k4a_transformation_depth_image_to_color_camera( g_hTransformation, hDepthImage, g_hTransformedDepthImage );

		if ( hDepthImage )
		{
			// イメージピクセルの先頭ポインタを取得する
			const uint16_t* pDepth = (uint16_t*) k4a_image_get_buffer( hDepthImage );
			if ( pDepth )
			{
				// イメージサイズを取得する
				uImageSize = (uint32_t) k4a_image_get_size( hDepthImage ) / sizeof(uint16_t);
				for( uint32_t u = 0; u < uImageSize; u++ )
				{
					const uint16_t w = pDepth[u];
					g_pdwPixel[0][u] = 0xFF000000 | (w << 16) | (w << 8) | w;
				}

				if ( rTransColorToDepth == K4A_RESULT_SUCCEEDED )
				{
					// 変換バッファを処理する
					const uint32_t* pTrans = (uint32_t*) k4a_image_get_buffer( g_hTransformedColorImage );
					if ( pTrans )
					{
						// イメージサイズを取得する
						uImageSize = (uint32_t) k4a_image_get_size( g_hTransformedColorImage ) / sizeof(uint32_t);
						for( uint32_t u = 0; u < uImageSize; u++ )
						{
							const uint16_t w = pDepth[u];
							if ( w )
								g_pdwPixel[2][u] = pTrans[u];
							else
								g_pdwPixel[2][u] = 0xFF000000;
						}
					}
				}
			}
		}

		if ( hColorImage )
		{
			// イメージピクセルの先頭ポインタを取得する
			const uint32_t* pColor = (uint32_t*) k4a_image_get_buffer( hColorImage );
			if ( pColor )
			{
				// イメージサイズを取得する
				uImageSize = (uint32_t) k4a_image_get_size( hColorImage );
				CopyMemory( g_pdwPixel[1], pColor, uImageSize );

				if ( rTransDepthToColor == K4A_RESULT_SUCCEEDED )
				{
					// 変換バッファを処理する
					const uint16_t* pTrans = (uint16_t*) k4a_image_get_buffer( g_hTransformedDepthImage );
					if ( pTrans )
					{
						// イメージサイズを取得する
						uImageSize = (uint32_t) k4a_image_get_size( g_hTransformedDepthImage ) / sizeof(uint16_t);
						for( uint32_t u = 0; u < uImageSize; u++ )
						{
							const uint16_t w = pTrans[u];
							if ( w )
								g_pdwPixel[3][u] = pColor[u];
							else
								g_pdwPixel[3][u] = 0xFF000000;
						}
					}
				}
			}
		}

		// キャプチャーを解放する
		k4a_capture_release( hCapture );

		// イメージを解放する
		if ( hDepthImage )
			k4a_image_release( hDepthImage );
		if ( hColorImage )
			k4a_image_release( hColorImage );
	}
	return uImageSize;
}

#if ENABLE_CSV_OUTPUT
// CSV ファイルにデータを出力する
void WriteCSV()
{
	HANDLE hFile = CreateFileA( "alignment.csv", GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL );
	if ( hFile != INVALID_HANDLE_VALUE )
	{
		for( int y = 0; y < DEPTH_RESOLUTION_HEIGHT; y++ )
		{
			// カラー情報を CSV に出力
			char szTmp[8];
			char szText[DEPTH_RESOLUTION_WIDTH * sizeof(szTmp)] = "";
			for( int x = 0; x < DEPTH_RESOLUTION_WIDTH; x++ )
			{
				const DWORD dw = g_pdwPixel[2][y * DEPTH_RESOLUTION_WIDTH + x];
				const BYTE c = ((dw & 0xFF) + ((dw >> 8) & 0xFF) + ((dw >> 16) & 0xFF)) / 3;
				sprintf_s( szTmp, 8, "%d,", c );
				strcat_s( szText, DEPTH_RESOLUTION_HEIGHT * sizeof(szTmp), szTmp );
			}

			// 改行してファイル出力
			strcat_s( szText, DEPTH_RESOLUTION_HEIGHT * sizeof(szTmp), "\r\n" );
			const DWORD dwLen = (DWORD) strlen( szText );
			DWORD dwWritten;
			WriteFile( hFile, szText, dwLen, &dwWritten, NULL );
		}
		CloseHandle( hFile );
	}
}
#endif

LRESULT CALLBACK WndProc( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam )
{
	switch( uMsg )
	{
	case WM_PAINT:
		{
			// 画面表示処理
			PAINTSTRUCT ps;
			HDC hDC = BeginPaint( hWnd, &ps );

			// 画面サイズを取得する
			RECT rect;
			GetClientRect( hWnd, &rect );

			// カラーの表示
			const int iBltWidth = rect.right /= 2;
			const int iBltHeight = rect.bottom /= 2;
			StretchBlt( hDC, 0, 0, iBltWidth, iBltHeight, g_hDCBMP[0], 0, 0, DEPTH_RESOLUTION_WIDTH, DEPTH_RESOLUTION_HEIGHT, SRCCOPY );
			StretchBlt( hDC, iBltWidth, 0, iBltWidth, iBltHeight, g_hDCBMP[1], 0, 0, COLOR_RESOLUTION_WIDTH, COLOR_RESOLUTION_HEIGHT, SRCCOPY );
			StretchBlt( hDC, 0, iBltHeight, iBltWidth, iBltHeight, g_hDCBMP[2], 0, 0, DEPTH_RESOLUTION_WIDTH, DEPTH_RESOLUTION_HEIGHT, SRCCOPY );
			StretchBlt( hDC, iBltWidth, iBltHeight, iBltWidth, iBltHeight, g_hDCBMP[3], 0, 0, COLOR_RESOLUTION_WIDTH, COLOR_RESOLUTION_HEIGHT, SRCCOPY );
			EndPaint( hWnd, &ps );
		}
		return 0;
#if ENABLE_CSV_OUTPUT
	case WM_KEYDOWN:
		// スペースキーが押されたら CSV 出力する
		if ( wParam == VK_SPACE )
			WriteCSV();
		break;
#endif
	case WM_CLOSE:
		DestroyWindow( hWnd );
	case WM_DESTROY:
		PostQuitMessage( 0 );
		break;
	default:
		return DefWindowProc( hWnd, uMsg, wParam, lParam );
	}
	return 0;
}

// アプリケーションの初期化 (ウィンドウや描画用のビットマップを作成)
HRESULT InitApp( HINSTANCE hInst, int nCmdShow )
{
	WNDCLASSEX wc = { 0, };
	wc.cbSize = sizeof(WNDCLASSEX);
	wc.style = CS_HREDRAW | CS_VREDRAW;
	wc.lpfnWndProc = WndProc;
	wc.hInstance = hInst;
	wc.hIcon = LoadIcon( NULL, IDI_APPLICATION );
	wc.hCursor = LoadCursor( NULL, IDC_ARROW );
	wc.hbrBackground = (HBRUSH) GetStockObject( NULL_BRUSH );
	wc.lpszClassName = szClassName;
	wc.hIconSm = LoadIcon( NULL, IDI_APPLICATION );
	if ( ! RegisterClassEx( &wc ) )
	{
		MessageBox( NULL, TEXT("アプリケーションクラスの初期化に失敗"), TEXT("エラー"), MB_OK );
		return E_FAIL;
	}

	// アプリケーションウィンドウを作成する
	g_hWnd = CreateWindow( szClassName, szClassName, WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, CW_USEDEFAULT, CW_USEDEFAULT, CW_USEDEFAULT, NULL, NULL, hInst, NULL );
	if ( ! g_hWnd )
	{
		MessageBox( NULL, TEXT("ウィンドウの初期化に失敗"), TEXT("エラー"), MB_OK );
		return E_FAIL;
	}

	// 画面表示用のビットマップを作成する
	ZeroMemory( &g_biBMP[0], sizeof(g_biBMP[0]) );
	g_biBMP[0].bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
	g_biBMP[0].bmiHeader.biBitCount = 32;
	g_biBMP[0].bmiHeader.biPlanes = 1;
	g_biBMP[0].bmiHeader.biWidth = DEPTH_RESOLUTION_WIDTH;
	g_biBMP[0].bmiHeader.biHeight = -(int) DEPTH_RESOLUTION_HEIGHT;
	g_hBMP[0] = CreateDIBSection( NULL, &g_biBMP[0], DIB_RGB_COLORS, (LPVOID*) (&g_pdwPixel[0]), NULL, 0 );

	ZeroMemory( &g_biBMP[1], sizeof(g_biBMP[1]) );
	g_biBMP[1].bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
	g_biBMP[1].bmiHeader.biBitCount = 32;
	g_biBMP[1].bmiHeader.biPlanes = 1;
	g_biBMP[1].bmiHeader.biWidth = COLOR_RESOLUTION_WIDTH;
	g_biBMP[1].bmiHeader.biHeight = -(int) COLOR_RESOLUTION_HEIGHT;
	g_hBMP[1] = CreateDIBSection( NULL, &g_biBMP[1], DIB_RGB_COLORS, (LPVOID*) (&g_pdwPixel[1]), NULL, 0 );

	ZeroMemory( &g_biBMP[2], sizeof(g_biBMP[2]) );
	g_biBMP[2].bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
	g_biBMP[2].bmiHeader.biBitCount = 32;
	g_biBMP[2].bmiHeader.biPlanes = 1;
	g_biBMP[2].bmiHeader.biWidth = DEPTH_RESOLUTION_WIDTH;
	g_biBMP[2].bmiHeader.biHeight = -(int) DEPTH_RESOLUTION_HEIGHT;
	g_hBMP[2] = CreateDIBSection( NULL, &g_biBMP[2], DIB_RGB_COLORS, (LPVOID*) (&g_pdwPixel[2]), NULL, 0 );

	ZeroMemory( &g_biBMP[3], sizeof(g_biBMP[3]) );
	g_biBMP[3].bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
	g_biBMP[3].bmiHeader.biBitCount = 32;
	g_biBMP[3].bmiHeader.biPlanes = 1;
	g_biBMP[3].bmiHeader.biWidth = COLOR_RESOLUTION_WIDTH;
	g_biBMP[3].bmiHeader.biHeight = -(int) COLOR_RESOLUTION_HEIGHT;
	g_hBMP[3] = CreateDIBSection( NULL, &g_biBMP[3], DIB_RGB_COLORS, (LPVOID*) (&g_pdwPixel[3]), NULL, 0 );

	HDC hDC = GetDC( g_hWnd );
	g_hDCBMP[0] = CreateCompatibleDC( hDC );
	g_hDCBMP[1] = CreateCompatibleDC( hDC );
	g_hDCBMP[2] = CreateCompatibleDC( hDC );
	g_hDCBMP[3] = CreateCompatibleDC( hDC );
	ReleaseDC( g_hWnd, hDC );
	g_hBMPold[0] = (HBITMAP) SelectObject( g_hDCBMP[0], g_hBMP[0] );
	g_hBMPold[1] = (HBITMAP) SelectObject( g_hDCBMP[1], g_hBMP[1] );
	g_hBMPold[2] = (HBITMAP) SelectObject( g_hDCBMP[2], g_hBMP[2] );
	g_hBMPold[3] = (HBITMAP) SelectObject( g_hDCBMP[3], g_hBMP[3] );

	ShowWindow( g_hWnd, nCmdShow );
	UpdateWindow( g_hWnd );

	return S_OK;
}

// アプリケーションの後始末
HRESULT UninitApp()
{
	// 画面表示用のビットマップを解放する
	for( int i = 0; i < 4; i++ )
	{
		if ( g_hDCBMP[i] || g_hBMP[i] )
		{
			SelectObject( g_hDCBMP[i], g_hBMPold[i] );
			DeleteObject( g_hBMP[i] );
			DeleteDC( g_hDCBMP[i] );
			g_hBMP[i] = NULL;
			g_hDCBMP[i] = NULL;
		}
	}
	return S_OK;
}

// エントリーポイント
int WINAPI WinMain( HINSTANCE hInst, HINSTANCE, LPSTR, int nCmdShow )
{
	// アプリケーションの初期化関数を呼ぶ
	if ( FAILED( InitApp( hInst, nCmdShow ) ) )
		return 1;

	// Kinect の初期化関数を呼ぶ
	if ( FAILED( CreateKinect() ) )
		return 1;

	// アプリケーションループ
	MSG msg;
	while( GetMessage( &msg, NULL, 0, 0 ) )
	{
		// ウィンドウメッセージを処理
		TranslateMessage( &msg );
		DispatchMessage( &msg );

		// Kinect 処理関数を呼ぶ
		if ( KinectProc() )
		{
			// Kinect 情報に更新があれば描画メッセージを発行する
			InvalidateRect( g_hWnd, NULL, TRUE );
		}
	}

	// Kinect の終了関数を呼ぶ
	DestroyKinect();

	// アプリケーションを終了関数を呼ぶ
	UninitApp();

	return (int) msg.wParam;
}
