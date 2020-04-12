#ifndef STRICT
#define STRICT	// �����ȃR�[�h���^��v������
#endif
#include <windows.h>
#include <tchar.h>
#include <stdio.h>
#include <k4a/k4a.h>

#pragma comment( lib, "k4a.lib" )

#define ENABLE_CSV_OUTPUT		1			// 1=CSV �o�͂�L���ɂ���

// �C���[�W�̉𑜓x
#define COLOR_RESOLUTION_WIDTH	(1280)
#define COLOR_RESOLUTION_HEIGHT	(720)
#define DEPTH_RESOLUTION_WIDTH	(640)
#define DEPTH_RESOLUTION_HEIGHT	(576)

// Win32 �A�v���p�̃p�����[�^
static const TCHAR szClassName[] = TEXT("�A���C�����g�T���v��");
HWND g_hWnd = NULL;							// �A�v���P�[�V�����̃E�B���h�E
HBITMAP g_hBMP[4] = { NULL, }, g_hBMPold[4] = { NULL, };	// �\������r�b�g�}�b�v�̃n���h��
HDC g_hDCBMP[4] = { NULL, };				// �\������r�b�g�}�b�v�̃R���e�L�X�g
BITMAPINFO g_biBMP[4] = { { 0, }, };		// �r�b�g�}�b�v�̏�� (�𑜓x��t�H�[�}�b�g)
LPDWORD g_pdwPixel[4] = { NULL, };			// �r�b�g�}�b�v�̒��g�̐擪 (�s�N�Z�����)

// Azure Kinect �p�̃p�����[�^
k4a_device_t g_hAzureKinect = nullptr;		// Azure Kinect �̃f�o�C�X�n���h��
k4a_transformation_t g_hTransformation = nullptr;
k4a_image_t g_hTransformedDepthImage = nullptr;
k4a_image_t g_hTransformedColorImage = nullptr;

// Kinect ������������
k4a_result_t CreateKinect()
{
	k4a_result_t hr;

	// Azure Kinect ������������
	hr = k4a_device_open( K4A_DEVICE_DEFAULT, &g_hAzureKinect );
	if ( hr == K4A_RESULT_SUCCEEDED )
	{
		// Azure Kinect �̃J�����ݒ�
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

		// �L�����u���[�V���������擾����
		k4a_calibration_t hCalibration;
		hr = k4a_device_get_calibration( g_hAzureKinect, config.depth_mode, config.color_resolution, &hCalibration );
		if ( hr == K4A_RESULT_SUCCEEDED )
		{
			// �ϊ��C���[�W���쐬����
			g_hTransformation = k4a_transformation_create( &hCalibration );
			hr = k4a_image_create( K4A_IMAGE_FORMAT_DEPTH16, COLOR_RESOLUTION_WIDTH, COLOR_RESOLUTION_HEIGHT, COLOR_RESOLUTION_WIDTH * sizeof(uint16_t), &g_hTransformedDepthImage );
			if ( hr == K4A_RESULT_SUCCEEDED )
			{
				hr = k4a_image_create( K4A_IMAGE_FORMAT_COLOR_BGRA32, DEPTH_RESOLUTION_WIDTH, DEPTH_RESOLUTION_HEIGHT, DEPTH_RESOLUTION_WIDTH * sizeof(uint32_t), &g_hTransformedColorImage );
				if ( hr == K4A_RESULT_SUCCEEDED )
				{
					// Azure Kinect �̎g�p���J�n����
					hr = k4a_device_start_cameras( g_hAzureKinect, &config );
					if ( hr == K4A_RESULT_SUCCEEDED )
					{
						return hr;
					}
				}
			}
		}

		// Azure Kinect �̎g�p����߂�
		MessageBox( NULL, TEXT("Azure Kinect ���J�n�ł��܂���ł���"), TEXT("�G���["), MB_OK );
		k4a_device_close( g_hAzureKinect );
	}
	else
	{
		MessageBox( NULL, TEXT("Azure Kinect �̏������Ɏ��s - �J�����̏�Ԃ��m�F���Ă�������"), TEXT("�G���["), MB_OK );
	}
	return hr;
}

// Kinect ���I������
void DestroyKinect()
{
	if ( g_hAzureKinect )
	{
		// Azure Kinect ���~����
		k4a_device_stop_cameras( g_hAzureKinect );

		// �ϊ��C���[�W���������
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

		// Azure Kinect �̎g�p����߂�
		k4a_device_close( g_hAzureKinect );
		g_hAzureKinect = nullptr;
	}
}

// Kinect �̃��C�����[�v����
uint32_t KinectProc()
{
	k4a_wait_result_t hr;
	uint32_t uImageSize = 0;

	// �L���v�`���[����
	k4a_capture_t hCapture = nullptr;
	hr = k4a_device_get_capture( g_hAzureKinect, &hCapture, K4A_WAIT_INFINITE );
	if ( hr == K4A_WAIT_RESULT_SUCCEEDED )
	{
		k4a_image_t hDepthImage, hColorImage;

		// �[�x�C���[�W���擾����
		hDepthImage = k4a_capture_get_depth_image( hCapture );

		// �J���[�C���[�W���擾����
		hColorImage = k4a_capture_get_color_image( hCapture );

		// �J���[����[�x�ɕϊ�����
		const k4a_result_t rTransColorToDepth = k4a_transformation_color_image_to_depth_camera( g_hTransformation, hDepthImage, hColorImage, g_hTransformedColorImage );

		// �[�x����J���[�ɕϊ�����
		const k4a_result_t rTransDepthToColor = k4a_transformation_depth_image_to_color_camera( g_hTransformation, hDepthImage, g_hTransformedDepthImage );

		if ( hDepthImage )
		{
			// �C���[�W�s�N�Z���̐擪�|�C���^���擾����
			const uint16_t* pDepth = (uint16_t*) k4a_image_get_buffer( hDepthImage );
			if ( pDepth )
			{
				// �C���[�W�T�C�Y���擾����
				uImageSize = (uint32_t) k4a_image_get_size( hDepthImage ) / sizeof(uint16_t);
				for( uint32_t u = 0; u < uImageSize; u++ )
				{
					const uint16_t w = pDepth[u];
					g_pdwPixel[0][u] = 0xFF000000 | (w << 16) | (w << 8) | w;
				}

				if ( rTransColorToDepth == K4A_RESULT_SUCCEEDED )
				{
					// �ϊ��o�b�t�@����������
					const uint32_t* pTrans = (uint32_t*) k4a_image_get_buffer( g_hTransformedColorImage );
					if ( pTrans )
					{
						// �C���[�W�T�C�Y���擾����
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
			// �C���[�W�s�N�Z���̐擪�|�C���^���擾����
			const uint32_t* pColor = (uint32_t*) k4a_image_get_buffer( hColorImage );
			if ( pColor )
			{
				// �C���[�W�T�C�Y���擾����
				uImageSize = (uint32_t) k4a_image_get_size( hColorImage );
				CopyMemory( g_pdwPixel[1], pColor, uImageSize );

				if ( rTransDepthToColor == K4A_RESULT_SUCCEEDED )
				{
					// �ϊ��o�b�t�@����������
					const uint16_t* pTrans = (uint16_t*) k4a_image_get_buffer( g_hTransformedDepthImage );
					if ( pTrans )
					{
						// �C���[�W�T�C�Y���擾����
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

		// �L���v�`���[���������
		k4a_capture_release( hCapture );

		// �C���[�W���������
		if ( hDepthImage )
			k4a_image_release( hDepthImage );
		if ( hColorImage )
			k4a_image_release( hColorImage );
	}
	return uImageSize;
}

#if ENABLE_CSV_OUTPUT
// CSV �t�@�C���Ƀf�[�^���o�͂���
void WriteCSV()
{
	HANDLE hFile = CreateFileA( "alignment.csv", GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL );
	if ( hFile != INVALID_HANDLE_VALUE )
	{
		for( int y = 0; y < DEPTH_RESOLUTION_HEIGHT; y++ )
		{
			// �J���[���� CSV �ɏo��
			char szTmp[8];
			char szText[DEPTH_RESOLUTION_WIDTH * sizeof(szTmp)] = "";
			for( int x = 0; x < DEPTH_RESOLUTION_WIDTH; x++ )
			{
				const DWORD dw = g_pdwPixel[2][y * DEPTH_RESOLUTION_WIDTH + x];
				const BYTE c = ((dw & 0xFF) + ((dw >> 8) & 0xFF) + ((dw >> 16) & 0xFF)) / 3;
				sprintf_s( szTmp, 8, "%d,", c );
				strcat_s( szText, DEPTH_RESOLUTION_HEIGHT * sizeof(szTmp), szTmp );
			}

			// ���s���ăt�@�C���o��
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
			// ��ʕ\������
			PAINTSTRUCT ps;
			HDC hDC = BeginPaint( hWnd, &ps );

			// ��ʃT�C�Y���擾����
			RECT rect;
			GetClientRect( hWnd, &rect );

			// �J���[�̕\��
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
		// �X�y�[�X�L�[�������ꂽ�� CSV �o�͂���
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

// �A�v���P�[�V�����̏����� (�E�B���h�E��`��p�̃r�b�g�}�b�v���쐬)
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
		MessageBox( NULL, TEXT("�A�v���P�[�V�����N���X�̏������Ɏ��s"), TEXT("�G���["), MB_OK );
		return E_FAIL;
	}

	// �A�v���P�[�V�����E�B���h�E���쐬����
	g_hWnd = CreateWindow( szClassName, szClassName, WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, CW_USEDEFAULT, CW_USEDEFAULT, CW_USEDEFAULT, NULL, NULL, hInst, NULL );
	if ( ! g_hWnd )
	{
		MessageBox( NULL, TEXT("�E�B���h�E�̏������Ɏ��s"), TEXT("�G���["), MB_OK );
		return E_FAIL;
	}

	// ��ʕ\���p�̃r�b�g�}�b�v���쐬����
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

// �A�v���P�[�V�����̌�n��
HRESULT UninitApp()
{
	// ��ʕ\���p�̃r�b�g�}�b�v���������
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

// �G���g���[�|�C���g
int WINAPI WinMain( HINSTANCE hInst, HINSTANCE, LPSTR, int nCmdShow )
{
	// �A�v���P�[�V�����̏������֐����Ă�
	if ( FAILED( InitApp( hInst, nCmdShow ) ) )
		return 1;

	// Kinect �̏������֐����Ă�
	if ( FAILED( CreateKinect() ) )
		return 1;

	// �A�v���P�[�V�������[�v
	MSG msg;
	while( GetMessage( &msg, NULL, 0, 0 ) )
	{
		// �E�B���h�E���b�Z�[�W������
		TranslateMessage( &msg );
		DispatchMessage( &msg );

		// Kinect �����֐����Ă�
		if ( KinectProc() )
		{
			// Kinect ���ɍX�V������Ε`�惁�b�Z�[�W�𔭍s����
			InvalidateRect( g_hWnd, NULL, TRUE );
		}
	}

	// Kinect �̏I���֐����Ă�
	DestroyKinect();

	// �A�v���P�[�V�������I���֐����Ă�
	UninitApp();

	return (int) msg.wParam;
}
