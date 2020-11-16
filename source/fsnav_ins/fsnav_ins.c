// заголовочные файлы стандартных библиотек C89
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

// заголовочные файлы библиотек лаборатории
#include "../../libs/fsnav.h"
#include "../../libs/ins/fsnav_ins_gravity.h"
#include "../../libs/ins/fsnav_ins_alignment.h"
#include "../../libs/ins/fsnav_ins_attitude.h"
#include "../../libs/ins/fsnav_ins_motion.h"

// проверка версии ядра
#define FSNAV_INS_FSNAV_BUS_VERSION_REQUIRED 8
#if FSNAV_BUS_VERSION < FSNAV_INS_FSNAV_BUS_VERSION_REQUIRED
	#error "fsnav bus version check failed, consider fetching the newest one"
#endif

#define FSNAV_INS_BUFFER_SIZE 1024

// частные алгоритмы приложения
void fsnav_ins_step_sync              (void);
void fsnav_ins_read_raw_input         (void);
void fsnav_ins_read_conv_input        (void);
void fsnav_ins_write_output           (void);
void fsnav_ins_print_progress         (void);
void fsnav_ins_compensate_static_drift(void);

void main(void)
{
	// конфигурационный файл
	const char cfgname[] = "fsnav_ins.cfg";
	
	FILE* fp;
	int i;
	char c;
	char* cfg;

	printf("fsnav_ins has started\n----\n");

	// открытие файла с конфигурацией
	fp = fopen(cfgname, "r");
	if (fp == NULL) {
		printf("error: couldn't open configuration file '%s'.\n", cfgname);
		return -1;
	}

	// определение размера кофигурационного файла
	fseek(fp, 0, SEEK_END);
	i = ftell(fp);
	if (i < 0) {
		printf("error: couldn't parse the size of configuration file '%s'.\n", cfgname);
		return -1;
	}
	fseek(fp, 0, SEEK_SET);
	if (i >= FSNAV_INS_BUFFER_SIZE) {
		printf("error: configuration '%s' contains more than allowed %d characters.\n", cfgname, FSNAV_INS_BUFFER_SIZE-1);
		return -1;
	}

	// выделение памяти под кофигурацию
	cfg = (char*)calloc((size_t)i+2, sizeof(char));
	if (cfg == NULL) {
		printf("error: couldn't allocate memory for the configuration.\n");
		return -1;
	}

	// считывание конфигурации
	i = 0;
	while (i < FSNAV_INS_BUFFER_SIZE-1 && (c = fgetc(fp)) != EOF) {
		cfg[i] = c;
		i++;
	}
	cfg[i] = '\0';
	fclose(fp);

	// добавление частных алгоритмов
	if (   !fsnav->add_plugin(fsnav_ins_step_sync              ) // ожидание метки времени шага навигационного решения
		|| !fsnav->add_plugin(fsnav_ins_read_raw_input         ) // считывание сырых показаний датчиков и их преобразование
		|| !fsnav->add_plugin(fsnav_ins_compensate_static_drift) // компенсация дрейфов гироскопов
		|| !fsnav->add_plugin(fsnav_ins_gravity_normal         ) // модель поля силы тяжести
		|| !fsnav->add_plugin(fsnav_ins_alignment_static       ) // начальная выставка
		|| !fsnav->add_plugin(fsnav_ins_attitude_rodrigues     ) // ориентация
		|| !fsnav->add_plugin(fsnav_ins_motion_euler           ) // положение и скорость
		|| !fsnav->add_plugin(fsnav_ins_motion_vertical_damping) // демпфирование в вертикальном канале
		|| !fsnav->add_plugin(fsnav_ins_write_output           ) // запись навигационного решения
		|| !fsnav->add_plugin(fsnav_ins_print_progress         ) // вывод на экран
		) {
		printf("error: couldn't add plugins.\n"); // ошибка добавления частных алгоритмов
		return;									
	}



	// инициализация ядра
	if (fsnav->init((char*)cfg))
		while(fsnav->step()); // основной цикл

	// ошибка инициализации
	else {
		printf("error: couldn't initialize.\n");
		return;
	}

	printf("\n----\nfsnav_ins has terminated\n");
}

/*
	шаг времени инерциального решения
	использует:
		не использует данные шины	
	изменяет:
		imu->t
		imu->w_valid
		imu->f_valid
	параметры:
		{imu: freq} — частота работы навигационного алгоритма, Гц
			тип: число
			диапазон: 50-3200
			значение по умолчанию: 100
			пример: {imu: freq = 400}
*/
void fsnav_ins_step_sync(void) 
{
	const char   freq_token[] = "freq";     // имя параметра в строке конфигурации, содержащего частоту
	const double freq_range[] = {50, 3200}; // диапазон допустимых частот
	const double freq_default = 100;        // частота по умолчанию

	static double        dt = -1;           // шаг по времени
	static unsigned long  i =  0;           // номер шага

	char *freq_ptr;                         // указатель на значение частоты в строке конфигурации

	// проверка инициализации инерциальной подсистемы на шине
	if (fsnav->imu == NULL)
		return;

	// инициализация
	if (fsnav->mode == 0) {
		// начальные значения
		fsnav->t = 0;
		i = 0;
		// поиск значения частоты в конфигурации
		freq_ptr = fsnav_locate_token(freq_token, fsnav->imu->cfg, fsnav->imu->cfglength, '=');
		if (freq_ptr != NULL)
			dt = atof(freq_ptr);	
		// если значение не найдено в конфигурации, или значение вне заданных пределов, установка по умолчанию
		if (freq_ptr == NULL || dt < freq_range[0] || freq_range[1] < dt)
			dt = freq_default;
		// вычисление шага по времени
		dt = 1 / dt;
	}

	// завершение работы
	else if (fsnav->mode < 0) {
		// ничего не делать
	}

	// шаг основного цикла	
	else {
		// сброс достоверности инерциальных датчиков
		fsnav->imu->w_valid = 0;
		fsnav->imu->f_valid = 0;
		// шаг по времени
		i++;
		fsnav->imu->t = i*dt;
	}
}

/*	
	чтение преобразованных показаний инерциальных датчиков из файла
	использует:
		не использует данные шины	
	изменяет:
		fsnav->imu.w
		fsnav->imu.w_valid
		fsnav->imu.f
		fsnav->imu.f_valid
	параметры:
		sensors_in — имя входного файла
			тип: строка
			пример: sensors_in = imu.txt
			без пробелов в имени
			с пробелом в конце
*/
void fsnav_ins_read_conv_input(void)
{
	const char input_file_token[] = "sensors_in"; // имя параметра конфигурации с входным файлом
	const int  n0 = 6;                            // требуемое количество параметров в строке входного файла
	
	static FILE *fp = NULL;                       // указатель на файл
	static char  buffer[FSNAV_INS_BUFFER_SIZE];    // строковый буфер

	char *cfg_ptr; // указатель на параметр в строке конфигурации
	int   n;       // количество параметров в строке


	// проверка инерциальной подсистемы на шине
	if (fsnav->imu == NULL)
		return;

	// инициализация
	if (fsnav->mode == 0) {
		// поиск имени входного файла в конфигурации
		cfg_ptr = fsnav_locate_token(input_file_token, fsnav->cfg_settings, fsnav->settings_length, '=');
		if (cfg_ptr != NULL)
			sscanf(cfg_ptr, "%s", buffer);
		// открытие файла
		fp = fopen(buffer, "r");
		if (fp == NULL) {
			printf("error: couldn't open input file '%s'.\n", buffer);
			fsnav->mode = -1;
			return;
		}
		// считывание заголовка
		fgets(buffer, FSNAV_INS_BUFFER_SIZE, fp);
		// обеспечить завершение строки нулевым символом
		buffer[FSNAV_INS_BUFFER_SIZE-1] = '\0';
	}

	// завершение работы
	else if (fsnav->mode < 0) {
		if (fp != NULL)
			fclose(fp); // закрытие файла, если он был открыт
		return;
	}

	// основной цикл
	else {
		// сброс флагов достоверности показаний датчиков
		fsnav->imu->w_valid = 0;
		fsnav->imu->f_valid = 0;
		// чтение строки из файла
		if (fgets(buffer, FSNAV_INS_BUFFER_SIZE, fp) == NULL) {
			fsnav->mode = -1; // завершение работы, если не удалось прочитать строку
			return;
		}
		// парсинг строки
		n = sscanf(buffer, "%lg %lg %lg %lg %lg %lg",
			&(fsnav->imu->w[0]), &(fsnav->imu->w[1]), &(fsnav->imu->w[2]),
			&(fsnav->imu->f[0]), &(fsnav->imu->f[1]), &(fsnav->imu->f[2]));
		if (n < n0) // недостаточно параметров в строке
			return;
		// перевод ДУС из градусов в радианы
		for (n = 0; n < 3; n++)
			fsnav->imu->w[n] /= fsnav->imu_const.rad2deg;
		// установка флагов достоверности
		fsnav->imu->w_valid = 1;
		fsnav->imu->f_valid = 1;
	}
}

/*	
	чтение сырых показаний инерциальных датчиков из файла
	использует:
		не использует данные шины	
	изменяет:
		fsnav->imu.w
		fsnav->imu.w_valid
		fsnav->imu.f
		fsnav->imu.f_valid
	параметры:
		sensors_in — имя входного файла
			тип: строка
			пример: sensors_in = imu.txt
			без пробелов в имени
			с пробелом в конце
*/
void fsnav_ins_read_raw_input(void)
{
	int i, j;

	const char input_file_token[] = "sensors_in"; // имя параметра конфигурации с входным файлом
	const int  n0 = 6;                            // требуемое количество параметров в строке входного файла
														  
	static FILE *fp = NULL;                       // указатель на файл
	static char  buffer[FSNAV_INS_BUFFER_SIZE];    // строковый буфер

	char       *cfg_ptr;        // указатель на параметр в строке конфигурации
	char       *tkn_ptr;        // указатель на токен в стоке файла
	const char  delim[] = ",;"; // разделители

	// масштабные коэффициенты
	const double w_scale = 0.00625;
	const double f_scale = 0.002447;

	// измерения
	static double w_raw[3];
	static double f_raw[3];

	// выходной файл
	static FILE *out = NULL;
	const int    num_col  = 6;         
	const char  *header[] = {"w1[d/s]", "w2[d/s]", "w3[d/s]", "f1[m/s^2]", "f2[m/s^2]", "f3[m/s^2]"};
	const int    fmt[]    = { 12,6,      12,6,      12,6,      12,6,        12,6,        12,6      };


	// проверка инерциальной подсистемы на шине
	if (fsnav->imu == NULL)
		return;

	// инициализация
	if (fsnav->mode == 0) {
		// поиск имени входного файла в конфигурации
		cfg_ptr = fsnav_locate_token(input_file_token, fsnav->cfg_settings, fsnav->settings_length, '=');
		if (cfg_ptr != NULL)
			sscanf(cfg_ptr, "%s", buffer);
		// открытие файла
		fp = fopen(buffer, "r");
		if (fp == NULL) {
			printf("error: couldn't open input file '%s'.\n", buffer);
			fsnav->mode = -1;
			return;
		}
		// считывание заголовка
		fgets(buffer, FSNAV_INS_BUFFER_SIZE, fp);
		// обеспечить завершение строки нулевым символом
		buffer[FSNAV_INS_BUFFER_SIZE-1] = '\0';

		// выходной файл
		out = fopen("sensors.txt", "w");
		if (out == NULL) {
			printf("error: couldn't open output file 'sensors.txt'.\n");
			fsnav->mode = -1;
			return;
		}
		// строка заголовка
		fprintf(out, " ");
		for (j = 0; j < num_col; j++)
			fprintf(out, "%-*s ", fmt[2*j], header[j]);
	}

	// завершение работы
	else if (fsnav->mode < 0) {
		if (fp != NULL)
			fclose(fp); // закрытие файла, если он был открыт
		return;
	}

	// основной цикл
	else {
		// сброс флагов достоверности показаний датчиков
		fsnav->imu->w_valid = 0;
		fsnav->imu->f_valid = 0;
		// чтение строки из файла
		if (fgets(buffer, FSNAV_INS_BUFFER_SIZE, fp) == NULL) {
			fsnav->mode = -1; // завершение работы, если не удалось прочитать строку
			return;
		}

		// парсинг строки
		// DIAG_STAT
		tkn_ptr = strtok(buffer, delim);
		// X_GYRO (16 bit)
		tkn_ptr = strtok(NULL, delim);
		w_raw[0] = (int16_t)atoi(tkn_ptr);
		// Y_GYRO (16 bit)
		tkn_ptr = strtok(NULL, delim);
		w_raw[1] = (int16_t)atoi(tkn_ptr);
		// Z_GYRO (16 bit)
		tkn_ptr = strtok(NULL, delim);
		w_raw[2] = (int16_t)atoi(tkn_ptr);
		// X_ACCL (16 bit)
		tkn_ptr = strtok(NULL, delim);
		f_raw[0] = (int16_t)atoi(tkn_ptr);
		// Y_ACCL (16 bit)
		tkn_ptr = strtok(NULL, delim);
		f_raw[1] = (int16_t)atoi(tkn_ptr);
		// Z_ACCL (16 bit)
		tkn_ptr = strtok(NULL, delim);
		f_raw[2] = (int16_t)atoi(tkn_ptr);

		// умножение на масштабный коэффициент + перевод в радианы
		for (i = 0; i < 3; i++) {
			w_raw[i] *= w_scale / fsnav->imu_const.rad2deg;
			f_raw[i] *= f_scale;
		}

		// перестановка осей
		fsnav->imu->w[0] =  w_raw[0];
		fsnav->imu->w[1] =  w_raw[2];
		fsnav->imu->w[2] = -w_raw[1];
		fsnav->imu->f[0] =  f_raw[0];
		fsnav->imu->f[1] =  f_raw[2];
		fsnav->imu->f[2] = -f_raw[1];

		// установка флагов достоверности
		fsnav->imu->w_valid = 1;
		fsnav->imu->f_valid = 1;

		// вывод в файл
		j = 0;
		fprintf(out, "\n");
		for (i = 0; i < 3; i++) fprintf(out, "%- *.*lf ", fmt[j], fmt[j+1], fsnav->imu->w[i]*fsnav->imu_const.rad2deg), j += 2;
		for (i = 0; i < 3; i++) fprintf(out, "%- *.*lf ", fmt[j], fmt[j+1], fsnav->imu->f[i]                        ), j += 2;
	}
}

/*
	запись навигационного решения в файл
	использует:
		fsnav->imu->t
		fsnav->imu.sol
	изменяет:
		не изменяет данные шины
	параметры:
		out — имя выходного файла
			тип: строка
			пример: {imu: out = ins.txt }
			без пробелов в имени
			с пробелом в конце
*/
void fsnav_ins_write_output(void)
{
	const char  nav_file_token[] = "out";

	// заголовок в выходном фале + количество выводимых символо всего и после запятой, для каждого параметра по порядку
	const int   num_col  = 10;         
	const char *header[] = {"time[s]", "lon[d]", "lat[d]", "hei[m]", "Ve[m/s]", "Vn[m/s]", "Vu[m/s]", "roll[d]", "pitch[d]", "heading[d]"};
	const int   fmt[]    = { 11,5,      15,8,     15,8,     10,3,      10,4,      10,4,      10,4,      13,8,      12,8,       13,8       };
	
	static FILE *fp = NULL;                    // указатель на файл
	static char  buffer[FSNAV_INS_BUFFER_SIZE]; // строковый буфер

	char *cfg_ptr; // указатель на параметр в строке конфигурации
	int   i, j;    // индексы

	// проверка инерциальной подсистемы на шине
	if (fsnav->imu == NULL)
		return;

	// инициализация частного алгоритма
	if (fsnav->mode == 0) {
		// поиск имени выходного файла в конфигурации
		cfg_ptr = fsnav_locate_token(nav_file_token, fsnav->cfg, fsnav->cfglength, '=');
		if (cfg_ptr != NULL)
			sscanf(cfg_ptr, "%s", buffer);
		// открытие файла
		fp = fopen(buffer, "w");
		if (fp == NULL) {
			printf("error: couldn't open output file '%s'.\n", buffer);
			fsnav->mode = -1;
			return;
		}		
		// строка заголовка
		fprintf(fp, " ");
		for (j = 0; j < num_col; j++)
			fprintf(fp, "%-*s ", fmt[2*j], header[j]);
	}

	// завершение работы
	else if (fsnav->mode < 0) {
		if (fp != NULL)
			fclose(fp); // закрытие файла, если он был открыт
		return;
	}

	// операции на каждом шаге
	else {
		// вывод навигационного решения в файл
		j = 0;
		                        fprintf(fp, "\n%- *.*lf ", fmt[j], fmt[j+1], fsnav->imu->t                                 ), j += 2;
		for (i = 0; i < 2; i++) fprintf(fp,   "%- *.*lf ", fmt[j], fmt[j+1], fsnav->imu->sol.llh[i]*fsnav->imu_const.rad2deg), j += 2;
		                        fprintf(fp,   "%- *.*lf ", fmt[j], fmt[j+1], fsnav->imu->sol.llh[2]                        ), j += 2;
		for (i = 0; i < 3; i++) fprintf(fp,   "%- *.*lf ", fmt[j], fmt[j+1], fsnav->imu->sol.v  [i]                        ), j += 2;
		for (i = 0; i < 3; i++) fprintf(fp,   "%- *.*lf ", fmt[j], fmt[j+1], fsnav->imu->sol.rpy[i]*fsnav->imu_const.rad2deg), j += 2;
	}
}

/*
	вывод текущего времени на экран
	использует:
		fsnav->imu->t
	изменяет:
		не изменяет данные шины
	параметры:
		нет использует параметры
*/
void fsnav_ins_print_progress(void)
{	
	const char bkspc[] = "\b\b\b\b\b"; // возврат курсора
	const int 
		decimals = sizeof(bkspc)-1,    // количество выводимых десятичных знаков целой части
		interval = 1024;               // интервал вывода, в шагах
	static long counter;               // счётчик

	// проверка инерциальной подсистемы на шине
	if (fsnav->imu == NULL)
		return;

	// инициализация частного алгоритма
	if (fsnav->mode == 0) {
		printf("seconds into navigation: % *.0f", decimals, fsnav->imu->t);
		counter = 0; // сброс счётчика
	}

	// завершение работы
	else if (fsnav->mode < 0)
		printf("%s% *.0f", bkspc, decimals, fsnav->imu->t); // печать конечного времени

	// операции на каждом шаге
	else {
		// печать на экран
		if (counter%interval == 0)
			printf("%s% *.0f", bkspc, decimals, fsnav->imu->t);
		counter++;
	}
}

/*
	компенсация дрейфов гироскопов
	использует:
		fsnav->imu->t
		fsnav->imu->w
		fsnav->imu->w_valid
	изменяет:
		не изменяет данные шины
	параметры:
		{imu: alignment} — длительность выставки, сек
			тип: число с плавающей точкой
			диапазон: +0 to +inf
			по умолчанию: 300
			пример: {imu: alignment = 900}
*/
void fsnav_ins_compensate_static_drift(void)
{
	const char   t0_token[] = "alignment"; // параметр длительности выставки в конфигурационной строке
	const double t0_default = 300;         // стандартная длительность выставки

	static double 
		w0[3] = {0,0,0}, // средняя угловая скорость измеряемая гироскопами
		t0      = -1;    // длительность выставки
	static int n = 0;    // сщетчик измерений

	char   *cfg_ptr; // указатель на параметр в строке конфигурации
	double  n1_n;    // (n - 1)/n
	size_t  i;       // индекс


	// проверка инерциальной подсистемы на шине
	if (fsnav->imu == NULL)
		return;

	// инициализация
	if (fsnav->mode == 0) {
		// обнуление счетчика
		n = 0;
		// парсинг длительности выставки в конфигурационной строке
		cfg_ptr = fsnav_locate_token(t0_token, fsnav->imu->cfg, fsnav->imu->cfglength, '=');
		if (cfg_ptr != NULL)
			t0 = atof(cfg_ptr);  
		if (cfg_ptr == NULL || t0 <= 0)
			t0 = t0_default;
	}
	
	// завершение работы
	else if (fsnav->mode < 0) {
		// ничего не делать
	}
	
	// операции на каждом шаге
	else {
		if (!(fsnav->imu->w_valid))
			return;
		
		if (fsnav->imu->t < t0) {
			// обновление среднего
			n++;
			n1_n = (n - 1.0)/n;
			for (i = 0; i < 3; i++)
				w0[i] = w0[i]*n1_n + fsnav->imu->w[i]/n;
		}
		else {
			// компенсация
			for (n = 0; n < 3; n++)
				fsnav->imu->w[n] -= w0[n];
		}
	}
}