// заголовочные файлы стандартных библиотек C89
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <float.h>
#include <math.h>
#include <string.h>

// заголовочные файлы библиотек лаборатории
#include "../../libs/fsnav.h"
#include "../../libs/ins/fsnav_ins_gravity.h"
#include "../../libs/ins/fsnav_ins_alignment.h"
#include "../../libs/ins/fsnav_ins_attitude.h"
#include "../../libs/ins/fsnav_ins_motion.h"

// проверка версии ядра
#define FSNAV_INS_FSNAV_BUS_VERSION_REQUIRED 12
#if FSNAV_BUS_VERSION < FSNAV_INS_FSNAV_BUS_VERSION_REQUIRED
	#error "fsnav bus version check failed, consider fetching the newest one"
#endif

#define FSNAV_INS_BUFFER_SIZE 4096
#define BIT16

// частные алгоритмы приложения
	// диспетчеризация
void fsnav_ins_scheduler(void);
	// ввод и вывод
void fsnav_ins_step_sync              (void);
void fsnav_ins_read_conv_input        (void);
void fsnav_ins_read_raw_input         (void);
void fsnav_ins_read_raw_input_temp    (void);
void fsnav_ins_write_output           (void);
void fsnav_ins_write_sensors          (void);
void fsnav_ins_switch_imu_axes        (void);
void fsnav_ins_print_progress         (void);
	// калибровка
void fsnav_ins_imu_calibration        (void);
void fsnav_ins_imu_calibration_temp   (void);
	// алгоритмы навигации
void fsnav_ins_compensate_static_drift(void);
void fsnav_ins_alignment_static_accs  (void);
void fsnav_ins_alignment_static_const (void);
void fsnav_ins_set_yaw_zero           (void);
void fsnav_ins_attitude_madgwick      (void);

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
		return;
	}

	// определение размера кофигурационного файла
	fseek(fp, 0, SEEK_END);
	i = ftell(fp);
	if (i < 0) {
		printf("error: couldn't parse the size of configuration file '%s'.\n", cfgname);
		return;
	}
	fseek(fp, 0, SEEK_SET);
	if (i >= FSNAV_INS_BUFFER_SIZE) {
		printf("error: configuration '%s' contains more than allowed %d characters.\n", cfgname, FSNAV_INS_BUFFER_SIZE-1);
		return;
	}

	// выделение памяти под кофигурацию
	cfg = (char*)calloc((size_t)i+2, sizeof(char));
	if (cfg == NULL) {
		printf("error: couldn't allocate memory for the configuration.\n");
		return;
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
	    || !fsnav->add_plugin(fsnav_ins_scheduler              ) // диспетчер
	    || !fsnav->add_plugin(fsnav_ins_read_raw_input_temp    ) // считывание сырых показаний датчиков, температуры и их преобразование
	    || !fsnav->add_plugin(fsnav_ins_imu_calibration_temp   ) // вычисление откалиброванных показаний датчиков (температурная модель)
	    || !fsnav->add_plugin(fsnav_ins_switch_imu_axes        ) // перестановка осей инерциальных датчиков
	    || !fsnav->add_plugin(fsnav_ins_write_sensors          ) // запись преобразованных показаний датчиков
	    || !fsnav->add_plugin(fsnav_ins_gravity_normal         ) // модель поля силы тяжести: стандартная
	    || !fsnav->add_plugin(fsnav_ins_gravity_constant       ) // модель поля силы тяжести: постоянная
	    || !fsnav->add_plugin(fsnav_ins_alignment_static       ) // начальная выставка: по акселерометрам и гироскопам
	    || !fsnav->add_plugin(fsnav_ins_alignment_static_accs  ) // начальная выставка: только по акселерометрам
	    || !fsnav->add_plugin(fsnav_ins_set_yaw_zero           ) // обнуление угла курса
	    || !fsnav->add_plugin(fsnav_ins_attitude_rodrigues     ) // ориентация
	    || !fsnav->add_plugin(fsnav_ins_attitude_madgwick      ) // фильтр Мэджвика
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





// диспетчеризация
	/*
		диспетчер выполнения плагинов и модификаций навигационного алгоритма
		использует:
			fsnav->imu->t
		изменяет:
			fsnav->imu_const (в случае выставления флагов)
		параметры:
			time_limit — ограничение по времени выполнения, сек
				тип: натуральное число
				диапазон: 0-DBL_MAX
				значение по умолчанию: DBL_MAX
				пример: time_limit = 360
			u_zero     — флаг обнуления угловой скорости земли в навигацонном алгоритме
			e2_zero    — флаг обнуления эксцентриситета в навигационном алгоритме
			g_const    — флаг постоянства силы тяжести
			accs_align — флаг выставки по акселерометрам
			yaw_zero   — флаг обнуления угла курса на этапе выставки
			madgwick_feedback_rate — параметр настройки фильтра Маджвика, радиан/сек
				тип: число с плавающей точкой
				диапазон: +0 до +inf
				пример: {imu: madgwick_feedback_rate = 0.003}
		примечание:
			флаги достаточно указать в конфигурацинной строке без указания значений
	*/
void fsnav_ins_scheduler(void)
{
	const char
		u_token       [] = "u_zero",     // имя параметра в строке конфигурации для обнуления угловой скорости земли
		e2_token      [] = "e2_zero",    // имя параметра в строке конфигурации для обнуления эксцентриситета
		g_token       [] = "g_const",    // имя параметра в строке конфигурации, определяющего режим счисления силы тяжести
		accs_token    [] = "accs_align", // имя параметра в строке конфигурации для выставки по акселерометрам
		yaw_token     [] = "yaw_zero";   // имя параметра в строке конфигурации для обнуления угла курса на этапе выставки
		
	static char yaw_zero; // флаг обнуления угла курса на этапе выставки

	const char    limit_token[] = "time_limit"; // имя параметра в строке конфигурации для ограничения по времени
	const double  limit_default = DBL_MAX;      // стандартное ограничение по времени (без ограничения), сек
	static double time_limit    = -1;

	const char    t0_token[] = "alignment"; // имя параметра в строке конфигурации с временем выставки
	const double  t0_default = 300;         // стандартное время выставки, сек
	static double t0         = -1;

	const char    madgwick_token[] = "madgwick_feedback_rate"; // имя параметра в строке конфигурации для счисления ориентации фильтром Мэджвика
	static double madgwick_rate    = -1;                       // параметр настройки фильтра Маджвика, рад/сек 

	char *cfg_ptr; // указатель на параметр в строке конфигурации

	// инициализация
	if (fsnav->mode == 0) {

#ifdef BIT16
	printf("16-bit mode of IMU measurements\n");
#else
	printf("32-bit mode of IMU measurements\n");
#endif

		// модификация констант на шине
			// поиск флага обнуления угловой скорости в конфигурации
		cfg_ptr = fsnav_locate_token(u_token, fsnav->cfg_settings, fsnav->settings_length, 0);
		if (cfg_ptr != NULL) {
			fsnav->imu_const.u = 0;
			printf("u_zero\n");
		}
			// поиск флага обнуления эксцентриситета в конфигурации
		cfg_ptr = fsnav_locate_token(e2_token, fsnav->cfg_settings, fsnav->settings_length, 0);
		if (cfg_ptr != NULL) {
			fsnav->imu_const.e2 = 0;
			printf("e2_zero\n");
		}
		
		// отключение плагинов в списке выполнения
			// поиск флага постоянста силы тяжести
		cfg_ptr = fsnav_locate_token(g_token, fsnav->cfg_settings, fsnav->settings_length, 0);
		if (cfg_ptr != NULL) {
			fsnav->suspend_plugin(fsnav_ins_gravity_normal);
			printf("g_const\n");
		}
		else
			fsnav->suspend_plugin(fsnav_ins_gravity_constant);
			// поиск флага счисления ориентации фильтром Маджвика
		cfg_ptr = fsnav_locate_token(madgwick_token, fsnav->imu->cfg, fsnav->imu->cfglength, '=');
		if (cfg_ptr != NULL) {
			madgwick_rate = atof(cfg_ptr); 
			if (madgwick_rate > 0 && isfinite(madgwick_rate)) {
				fsnav->suspend_plugin(fsnav_ins_attitude_rodrigues);
				printf("%s = %g\n", madgwick_token, madgwick_rate);
			}
			else
				fsnav->suspend_plugin(fsnav_ins_attitude_madgwick);
		}
		else
			fsnav->suspend_plugin(fsnav_ins_attitude_madgwick);
			// поиск флага выставки по акселерометрам
		cfg_ptr = fsnav_locate_token(accs_token, fsnav->cfg_settings, fsnav->settings_length, 0);
		if (cfg_ptr != NULL) {
			fsnav->suspend_plugin(fsnav_ins_alignment_static);
			printf("accs_align\n");
		}
		else
			fsnav->suspend_plugin(fsnav_ins_alignment_static_accs);
			// поиск флага обнуления угла курса на этапе выставки
		cfg_ptr = fsnav_locate_token(yaw_token, fsnav->cfg_settings, fsnav->settings_length, 0);
		if (cfg_ptr != NULL) {
			yaw_zero = 1;
			printf("yaw_zero\n");
		}
		else {
			yaw_zero = 0;
			fsnav->suspend_plugin(fsnav_ins_set_yaw_zero);
		}

		// временные параметры
			// поиск ограничения по времени в конфигурации
		cfg_ptr = fsnav_locate_token(limit_token, fsnav->cfg_settings, fsnav->settings_length, '=');
		if (cfg_ptr != NULL)
			time_limit = atof(cfg_ptr);
		if (cfg_ptr == NULL || time_limit <= 0)
			time_limit = limit_default;
			// поиск времени выставки в конфигурации
		cfg_ptr = fsnav_locate_token(t0_token, fsnav->imu->cfg, fsnav->imu->cfglength, '=');
		if (cfg_ptr != NULL)
			t0 = atof(cfg_ptr);
		if (cfg_ptr == NULL || t0 <= 0)
			t0 = t0_default;
	}

	// завершение работы
	else if (fsnav->mode < 0) {}

	// шаг основного цикла
	else {
		if (fsnav->imu->t > time_limit)
			fsnav->mode = -1;

		if (fsnav->imu->t > t0 && yaw_zero) {
			fsnav->suspend_plugin(fsnav_ins_set_yaw_zero);
			yaw_zero = 0;
		}
	}
}





// ввод и вывод
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

	static double        dt   = -1;         // шаг по времени
	static unsigned long i    =  0;         // номер шага

	char *cfg_ptr;                          // указатель на строку конфигурации

	// проверка инициализации инерциальной подсистемы на шине
	if (fsnav->imu == NULL)
		return;

	// инициализация
	if (fsnav->mode == 0) {
		// начальные значения
		fsnav->t = 0;
		i = 0;
		// поиск значения частоты в конфигурации
		cfg_ptr = fsnav_locate_token(freq_token, fsnav->imu->cfg, fsnav->imu->cfglength, '=');
		if (cfg_ptr != NULL)
			dt = atof(cfg_ptr);	
		// если значение не найдено в конфигурации, или значение вне заданных пределов, установка по умолчанию
		if (cfg_ptr == NULL || dt < freq_range[0] || freq_range[1] < dt)
			dt = freq_default;
		// вычисление шага по времени
		dt = 1 / dt;
	}

	// завершение работы
	else if (fsnav->mode < 0) {}

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
		чтение сырых показаний инерциальных датчиков ADIS16505-1 из файла
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
	int i;

	const char input_file_token[] = "sensors_in"; // имя параметра конфигурации с входным файлом
	const int  n0 = 6;                            // требуемое количество параметров в строке входного файла
														  
	static FILE *fp = NULL;                       // указатель на файл
	static char  buffer[FSNAV_INS_BUFFER_SIZE];    // строковый буфер

	char       *cfg_ptr;        // указатель на параметр в строке конфигурации
	char       *tkn_ptr;        // указатель на токен в стоке файла
	const char  delim[] = ",;"; // разделители

	// масштабные коэффициенты
#ifdef BIT16
	const double w_scale = 0.00625;
	const double f_scale = 0.002447;
#else
	const double w_scale = 0.00625/pow(2,16);
	const double f_scale = 0.002447/pow(2,16);
#endif
	const double T_scale = 0.1;

	// измерения
	static int w_raw[3];
	static int f_raw[3];

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
		// DIAG_STAT
		tkn_ptr = strtok(buffer, delim);
		// X_GYRO
		tkn_ptr = strtok(NULL, delim);
		w_raw[0] = atoi(tkn_ptr);
		// Y_GYRO
		tkn_ptr = strtok(NULL, delim);
		w_raw[1] = atoi(tkn_ptr);
		// Z_GYRO
		tkn_ptr = strtok(NULL, delim);
		w_raw[2] = atoi(tkn_ptr);
		// X_ACCL
		tkn_ptr = strtok(NULL, delim);
		f_raw[0] = atoi(tkn_ptr);
		// Y_ACCL
		tkn_ptr = strtok(NULL, delim);
		f_raw[1] = atoi(tkn_ptr);
		// Z_ACCL
		tkn_ptr = strtok(NULL, delim);
		f_raw[2] = atoi(tkn_ptr);

#ifdef BIT16
		for (i = 0; i < 3; i++) {
			w_raw[i] = (int16_t)w_raw[i];
			f_raw[i] = (int16_t)f_raw[i];
		}
#else
		for (i = 0; i < 3; i++) {
			w_raw[i] = (int32_t)w_raw[i];
			f_raw[i] = (int32_t)f_raw[i];
		}
#endif

		// умножение на масштабный коэффициент + перевод в радианы
		for (i = 0; i < 3; i++) {
			fsnav->imu->w[i] = w_raw[i] * w_scale / fsnav->imu_const.rad2deg;
			fsnav->imu->f[i] = f_raw[i] * f_scale;
		}

		// установка флагов достоверности
		fsnav->imu->w_valid = 1;
		fsnav->imu->f_valid = 1;
	}
}

	/*	
		чтение сырых показаний инерциальных датчиков ADIS16505-1 вместе с температурой из файла
		использует:
			не использует данные шины	
		изменяет:
			fsnav->imu.w
			fsnav->imu.w_valid
			fsnav->imu.f
			fsnav->imu.f_valid
			fsnav->imu.T
			fsnav->imu.T_valid
		параметры:
			sensors_in — имя входного файла
				тип: строка
				пример: sensors_in = imu.txt
				без пробелов в имени
				с пробелом в конце
	*/
void fsnav_ins_read_raw_input_temp(void)
{
	size_t i;

	const char input_file_token[] = "sensors_in"; // имя параметра конфигурации с входным файлом
	const int  n0 = 7;                            // требуемое количество параметров в строке входного файла
														  
	static FILE *fp = NULL;                       // указатель на файл
	static char  buffer[FSNAV_INS_BUFFER_SIZE];    // строковый буфер

	char       *cfg_ptr;        // указатель на параметр в строке конфигурации
	char       *tkn_ptr;        // указатель на токен в стоке файла
	const char  delim[] = ",;"; // разделители

	// масштабные коэффициенты
#ifdef BIT16
	const double w_scale = 0.00625;
	const double f_scale = 0.002447;
#else
	const double w_scale = 0.00625/pow(2,16);
	const double f_scale = 0.002447/pow(2,16);
#endif
	const double T_scale = 0.1;

	// измерения
	static int w_raw[3];
	static int f_raw[3];
	static int T;

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
		// DIAG_STAT
		tkn_ptr = strtok(buffer, delim);
		// X_GYRO
		tkn_ptr = strtok(NULL, delim);
		w_raw[0] = atoi(tkn_ptr);
		// Y_GYRO
		tkn_ptr = strtok(NULL, delim);
		w_raw[1] = atoi(tkn_ptr);
		// Z_GYRO
		tkn_ptr = strtok(NULL, delim);
		w_raw[2] = atoi(tkn_ptr);
		// X_ACCL
		tkn_ptr = strtok(NULL, delim);
		f_raw[0] = atoi(tkn_ptr);
		// Y_ACCL
		tkn_ptr = strtok(NULL, delim);
		f_raw[1] = atoi(tkn_ptr);
		// Z_ACCL
		tkn_ptr = strtok(NULL, delim);
		f_raw[2] = atoi(tkn_ptr);
		// TEMP_OUT
		tkn_ptr = strtok(NULL, delim);
		T = (int)atoi(tkn_ptr);

#ifdef BIT16
		for (i = 0; i < 3; i++) {
			w_raw[i] = (int16_t)w_raw[i];
			f_raw[i] = (int16_t)f_raw[i];
		}
#else
		for (i = 0; i < 3; i++) {
			w_raw[i] = (int32_t)w_raw[i];
			f_raw[i] = (int32_t)f_raw[i];
		}
#endif

		// умножение на масштабный коэффициент + перевод в радианы
		for (i = 0; i < 3; i++) {
			fsnav->imu->w [i] = w_raw[i] * w_scale / fsnav->imu_const.rad2deg;
			fsnav->imu->f [i] = f_raw[i] * f_scale;
			fsnav->imu->Tw[i] = T * T_scale;
			fsnav->imu->Tf[i] = T * T_scale;
		}

		// установка флагов достоверности
		fsnav->imu->w_valid  = 1;
		fsnav->imu->f_valid  = 1;
		fsnav->imu->Tw_valid = 1;
		fsnav->imu->Tf_valid = 1;
	}
}

	/*
		запись показаний датчиков в файл
		использует:
			fsnav->imu->f
			fsnav->imu->w
		изменяет:
			не изменяет данные шины
		параметры:
			sensors_out — имя выходного файла
				тип: строка
				пример: {imu: sensors_out = sensors.txt }
				без пробелов в имени
				с пробелом в конце
	*/
void fsnav_ins_write_sensors(void)
{
	const char  sensors_file_token[] = "sensors_out";

	// заголовок в выходном файле + количество выводимых символов всего и после запятой, для каждого параметра по порядку
	const int   num_col  = 6;         
	const char *header[] = {"w1[d/s]", "w2[d/s]", "w3[d/s]", "f1[m/s^2]", "f2[m/s^2]", "f3[m/s^2]"};
	const int   fmt[]    = { 12,6,      12,6,      12,6,      12,6,        12,6,        12,6      };

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
		cfg_ptr = fsnav_locate_token(sensors_file_token, fsnav->cfg, fsnav->cfglength, '=');
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
		fprintf(fp, "%%");
		for (j = 0; j < num_col; j++)
			fprintf(fp, "%-*s ", fmt[2*j], header[j]);
	}

	// завершение работы
	else if (fsnav->mode < 0) {
		if (fp != NULL)
			fclose(fp);
		return;
	}

	// операции на каждом шаге
	else {
		j = 0;
		fprintf(fp, "\n");
		for (i = 0; i < 3; i++) fprintf(fp, "%- *.*lf ", fmt[j], fmt[j+1], fsnav->imu->w[i]*fsnav->imu_const.rad2deg), j += 2;
		for (i = 0; i < 3; i++) fprintf(fp, "%- *.*lf ", fmt[j], fmt[j+1], fsnav->imu->f[i]                        ), j += 2;
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
	const char  nav_file_token[] = "nav_out";

	// заголовок в выходном фале + количество выводимых символо всего и после запятой, для каждого параметра по порядку
	const int   num_col  = 10;         
	const char *header[] = {"time[s]", "lon[d]", "lat[d]", "hei[m]", "Ve[m/s]", "Vn[m/s]", "Vu[m/s]", "roll[d]", "pitch[d]", "heading[d]"};
	const int   fmt[]    = { 11,5,      15,8,     15,8,     10,3,     10,4,      10,4,      10,4,      13,8,      12,8,       13,8       };
	
	static FILE *fp = NULL;                    // указатель на файл
	static char  buffer[FSNAV_INS_BUFFER_SIZE]; // строковый буфер

	char *cfg_ptr; // указатель на параметр в строке конфигурации
	int   i, j;    // индексы

	// проверка инерциальной подсистемы на шине
	if (fsnav->imu == NULL)
		return;

	// инициализация
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
		fprintf(fp, "%%");
		for (j = 0; j < num_col; j++)
			fprintf(fp, "%-*s ", fmt[2*j], header[j]);
	}

	// завершение работы
	else if (fsnav->mode < 0) {
		if (fp != NULL)
			fclose(fp);
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
		перестановка осей интерциальных датчиков к системе координат: первая ось — продольная, вторая ось — вертикальная, третья ось — по правому крылу.
		использует:
			fsnav->imu->f
			fsnav->imu->w
		изменяет:
			fsnav->imu->f
			fsnav->imu->w
		параметры:
			не использует параметры
	*/
void fsnav_ins_switch_imu_axes(void)
{
	size_t i;
	static double buf[3] = {0};
	static double A  [9] = {0, 1, 0, 0, 0, 1, 1, 0, 0};

	// проверка инерциальной подсистемы на шине
	if (fsnav->imu == NULL)
		return;
	
	// инициализация
	if (fsnav->mode == 0) {}

	// завершение работы
	else if (fsnav->mode < 0) {}
	
	// операции на каждом шаге
	else {
		// гироскопы
		for (i = 0; i < 3; i++)
			buf[i] = fsnav->imu->w[i];
		fsnav_linal_mmul(fsnav->imu->w, &A[0], &buf[0], 3, 3, 1);

		// акселерометры
		for (i = 0; i < 3; i++)
			buf[i] = fsnav->imu->f[i];
		fsnav_linal_mmul(fsnav->imu->f, &A[0], &buf[0], 3, 3, 1);
	}
}

	/*
		вывод текущего времени на экран
		использует:
			fsnav->imu->t
		изменяет:
			не изменяет данные шины
		параметры:
			не использует параметры
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





// калибровка
	/*
		вычисление откалиброванных показаний датчиков
		использует:
			fsnav->imu->w
			fsnav->imu->w_valid
			fsnav->imu->f
			fsnav->imu->f_valid
		изменяет:
			fsnav->imu->w
			fsnav->imu->f
		параметры:
			{imu: df01, df02, df03} — смещения нулей акселерометров
				тип: числа с плавающей точкой
				диапазон: -inf до +inf
				по умолчанию: 0
				пример: {imu: df01 = −0.0152150, df02 = −0.0077081, df03 = +0.0119966}
			{imu: ga11, ga22, ga33} — масштабные коэффициенты акселерометров
				тип: числа с плавающей точкой
				диапазон: -inf до +inf
				по умолчанию: 0
				пример: {imu: ga11 = -0.0028193, ga22 = -0.0017478, ga33 = +0.0004237}
			{imu: nu01, nu02, nu03} — смещения нулей гироскопов
				тип: числа с плавающей точкой
				диапазон: -inf до +inf
				по умолчанию: 0
				пример: {imu: nu01 = −209.229411614, nu02 = +1395.88240499, nu03 = −259.620225000}
			{imu: theta11, theta22, theta33} — масштабные коэффициенты гироскопов
				тип: числа с плавающей точкой
				диапазон: -inf до +inf
				по умолчанию: 0
				пример: {imu: th11 = +0.02477090, th22 = +0.02723770, th33 = +0.02404710}
	*/
void fsnav_ins_imu_calibration(void)
{
	size_t      i;
	char*       cfg_ptr;
	const char* tokens[12] = {"df01", "df02", "df03", "ga11", "ga22", "ga33", "nu01", "nu02", "nu03", "th11", "th22", "th33"};

	// калибровочные коэффициенты
	static double nu0  [3] = {0};
	static double Theta[3] = {0};
	static double df0  [3] = {0}; 
	static double Gamma[3] = {0};

	// проверка инерциальной подсистемы на шине
	if (fsnav->imu == NULL)
		return;

	// инициализация
	if (fsnav->mode == 0) {
		// считывание калибровочных коэффициентов
		for (i = 0; i < 3; i++) {
			cfg_ptr = fsnav_locate_token(tokens[i], fsnav->imu->cfg, fsnav->imu->cfglength, '=');
			if (cfg_ptr != NULL)
				df0[i] = atof(cfg_ptr);
		}
		for (i = 0; i < 3; i++) {
			cfg_ptr = fsnav_locate_token(tokens[i+3], fsnav->imu->cfg, fsnav->imu->cfglength, '=');
			if (cfg_ptr != NULL)
				Gamma[i] = atof(cfg_ptr);
		}
		for (i = 0; i < 3; i++) {
			cfg_ptr = fsnav_locate_token(tokens[i+6], fsnav->imu->cfg, fsnav->imu->cfglength, '=');
			if (cfg_ptr != NULL)
				nu0[i] = atof(cfg_ptr);
		}
		for (i = 0; i < 3; i++) {
			cfg_ptr = fsnav_locate_token(tokens[i+9], fsnav->imu->cfg, fsnav->imu->cfglength, '=');
			if (cfg_ptr != NULL)
				Theta[i] = atof(cfg_ptr);
		}

		// переход в СИ: град/час -> рад/сек
		for (i = 0; i < 3; i++) {
			nu0[i] /= fsnav->imu_const.rad2deg;
			nu0[i] /= 3600.0;
		}
	}

	// завершение работы
	else if (fsnav->mode < 0) {}

	// операции на каждом шаге
	else {
		for (i = 0; i < 3; i++) {
			fsnav->imu->f[i] -= df0[i];
			fsnav->imu->f[i] /= (1 + Gamma[i]);
			fsnav->imu->w[i] -= nu0[i];
			fsnav->imu->w[i] /= (1 + Theta[i]);
		}
	}
}

	/*
		вычисление откалиброванных показаний датчиков (по температурной модели)
		использует:
			fsnav->imu->w
			fsnav->imu->w_valid
			fsnav->imu->f
			fsnav->imu->f_valid
			fsnav->imu->T
			fsnav->imu->T_valid
		изменяет:
			fsnav->imu->w
			fsnav->imu->f
		параметры:
			{imu: df0i_a0, df0i_a1, df0i_a2} — коэффициенты аппроксимации смещения нулей акселерометров (i = 1,2,3)
				тип: числа с плавающей точкой
				диапазон: -inf до +inf
				по умолчанию: 0
				пример: {imu: df01_a0 = −0.017157, df01_a1 = +0.000324, df01_a2 = −0.000005}
			{imu: df0i_T0, df0i_T1} — температурный диапазон [T0,T1] аппроксимации смещения нулей акселерометров (i = 1,2,3)
				тип: числа с плавающей точкой
				диапазон: -inf до +inf
				по умолчанию: 30
				пример: {imu: df01_T0 = 15.4, df01_T1 = 25.0}
			{imu: ga11, ga21, ga22, ga31, ga32, ga33} — масштабные коэффициенты и перекосы для акселерометров
				тип: числа с плавающей точкой
				диапазон: -inf до +inf
				по умолчанию: 0
				пример: {imu: ga11 = 0.00133332, ga21 = -0.00153002, ga31 = -0.00340478, ga22 = 0.00019859, ga32 = -0.0011622, ga33 = 0.00277688}
			{imu: nu0i_a1, nu0i_a2} — коэффициенты аппроксимации смещения нулей гироскопов (i = 1,2,3)
				тип: числа с плавающей точкой
				диапазон: -inf до +inf
				по умолчанию: 0
				пример: {imu: nu01_a1 = −32.050358, nu01_a2 = +1.045406}
			{imu: nu0i_T0, nu0i_T1} — температурный диапазон [T0,T1] аппроксимации смещения нулей гироскопов (i = 1,2,3)
				тип: числа с плавающей точкой
				диапазон: -inf до +inf
				по умолчанию: 30
				пример: {imu: nu01_T0 = 18.1, nu01_T1 = 27.3}
			{imu: th11, th12, th13, th21, th22, th23, th31, th32, th33} — масштабные коэффициенты и перекосы для гироскопов
				тип: числа с плавающей точкой
				диапазон: -inf до +inf
				по умолчанию: 0
				пример: {imu: th11 = 0.00017579, th12 = 0.00145691, th13 = 0.00180267, th21 = -0.00076913, th22 = -0.00033748, th23 = 0.00406421, th31 = -0.00022133, th32 = -0.0003247, th33 = 0.00004347}
			{imu: d11, d12, d13, d21, d22, d23, d31, d32, d33} — коэффициенты динамических дрейфов гироскопов
				тип: числа с плавающей точкой
				диапазон: -inf до +inf
				по умолчанию: 0
				пример: {imu: d11 = 0.00474286, d12 = -0.00399136, d13 = -0.01231876, d21 = -0.01289159, d22 = -0.00676363, d23 = 0.01033997, d31 = -0.00337131, d32 = 0.00383757, d33 = -0.00373017}
	*/
void fsnav_ins_imu_calibration_temp(void)
{
	size_t      i;
	char*       cfg_ptr;

	// осреднение дрейфов гироскопов на выставке
	const char   t0_token[] = "alignment"; // параметр длительности выставки в конфигурационной строке
	const double t0_default = 300;         // стандартная длительность выставки

	static double 
		w0[3]  = {0}, // средняя угловая скорость измеряемая гироскопами
		Tw0[3] = {0}, // средняя температура гироскопов на выставке
		t0    = -1;   // длительность выставки
	static int n = 0; // счетчик измерений
	double     n1_n;  // (n-1)/n

	// токены конфигурационного файла для коэффициентов калибровки
	const char* nu0_tokens  [6] = {"nu01_a1", "nu01_a2", "nu02_a1", "nu02_a2", "nu03_a1", "nu03_a2"};
	const char* df0_tokens  [9] = {"df01_a0", "df01_a1", "df01_a2", "df02_a0", "df02_a1", "df02_a2", "df03_a0", "df03_a1", "df03_a2"};
	const char* Gamma_tokens[6] = {"ga11", "ga21", "ga31", "ga22", "ga32", "ga33"};
	const char* Theta_tokens[9] = {"th11", "th12", "th13", "th21", "th22", "th23", "th31", "th32", "th33"};
	const char* D_tokens    [9] = { "d11",  "d12",  "d13",  "d21",  "d22",  "d23",  "d31",  "d32",  "d33"};

	// коэффициенты калибровки
	static double nu0_app[6] = {0}; // {nu01_a1, nu01_a2, nu02_a1, nu02_a2, nu03_a1, nu03_a2}
	static double df0_app[9] = {0}; // {df01_a0, df01_a1, df01_a2, df02_a0, df02_a1, df02_a2, df03_a0, df03_a1, df03_a2}
	static double Gamma  [6] = {0}; // {ga11, ga21, ga31, ga22, ga32, ga33}
	static double Theta  [9] = {0}; // {th11, th12, th13, th21, th22, th23, th31, th32, th33}
	static double D      [9] = {0}; // { d11,  d12,  d13,  d21,  d22,  d23,  d31,  d32,  d33}

	// погрешности
	static double nu0[3] = {0};
	static double df0[3] = {0};
	static double nu [3] = {0};
	static double df [3] = {0};

	// f/g
	static double f_g[3] = {0};

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
		// считывание коэффициентов калибровки
			// температурная модель дрейфов акселерометров
		for (i = 0; i < 9; i++) {
			cfg_ptr = fsnav_locate_token(df0_tokens[i], fsnav->imu->cfg, fsnav->imu->cfglength, '=');
			if (cfg_ptr != NULL)
				df0_app[i] = atof(cfg_ptr);
			else
				df0_app[i] = 0.0;
		}
			// перекосы и масштабы акселерометров
		for (i = 0; i < 6; i++) {
			cfg_ptr = fsnav_locate_token(Gamma_tokens[i], fsnav->imu->cfg, fsnav->imu->cfglength, '=');
			if (cfg_ptr != NULL)
				Gamma[i] = atof(cfg_ptr);
			else
				Gamma[i] = 0.0;
		}
			// температурная модель дрейфов гироскопов
		for (i = 0; i < 6; i++) {
			cfg_ptr = fsnav_locate_token(nu0_tokens[i], fsnav->imu->cfg, fsnav->imu->cfglength, '=');
			if (cfg_ptr != NULL)
				nu0_app[i] = atof(cfg_ptr);
			else
				nu0_app[i] = 0.0;
		}
			// перекосы и масштабы гироскопов
		for (i = 0; i < 9; i++) {
			cfg_ptr = fsnav_locate_token(Theta_tokens[i], fsnav->imu->cfg, fsnav->imu->cfglength, '=');
			if (cfg_ptr != NULL)
				Theta[i] = atof(cfg_ptr);
			else
				Theta[i] = 0.0;
		}
			// динамические дрейфы гироскопов
		for (i = 0; i < 9; i++) {
			cfg_ptr = fsnav_locate_token(D_tokens[i], fsnav->imu->cfg, fsnav->imu->cfglength, '=');
			if (cfg_ptr != NULL)
				D[i] = atof(cfg_ptr);
			else
				D[i] = 0.0;
		}
		// переход в СИ: град/час -> рад/сек
		for (i = 0; i < 6; i++) {
			nu0_app[i] /= fsnav->imu_const.rad2deg;
			nu0_app[i] /= 3600.0;
		}
		// переход в СИ: град/сек -> рад/сек
		for (i = 0; i < 9; i++)
			D[i] /= fsnav->imu_const.rad2deg;
	}

	// завершение работы
	else if (fsnav->mode < 0) {}

	// операции на каждом шаге
	else {
		if (!(fsnav->imu->w_valid))
			return;

		// калибровка гироскопов
			// вычитание ошибки, связанной с перекосами и масштабами
		fsnav_linal_mmul(&nu[0], &Theta[0], fsnav->imu->w, 3, 3, 1);
		for (i = 0; i < 3; i++)
			fsnav->imu->w[i] -= nu[i];
			// вычитание динамических дрейфов
		for (i = 0; i < 3; i++)
			f_g[i] =  fsnav->imu->f[i]/fabs(fsnav->imu->g[2]);
		fsnav_linal_mmul(&nu[0], &D[0], &f_g[0], 3, 3, 1);
		for (i = 0; i < 3; i++)
			fsnav->imu->w[i] -= nu[i];

		if (fsnav->imu->t < t0) {
			// обновление среднего гироскопов и температуры
			n++;
			n1_n = (n - 1.0)/n;
			for (i = 0; i < 3; i++) {
				w0 [i] = w0 [i]*n1_n + fsnav->imu->w [i]/n;
				Tw0[i] = Tw0[i]*n1_n + fsnav->imu->Tw[i]/n;
			}
		}
		else {
			// вычитание статических дрейфов
			for (i = 0; i < 3; i++) {
				nu0[i]  = w0[i] - nu0_app[2*i]*Tw0[i] - nu0_app[2*i+1]*Tw0[i]*Tw0[i];
				nu0[i] += nu0_app[2*i  ]*fsnav->imu->Tw[i];
				nu0[i] += nu0_app[2*i+1]*fsnav->imu->Tw[i]*fsnav->imu->Tw[i];
					
				fsnav->imu->w[i] -= nu0[i];
			}
		}

		// калибровка акселерометров
			// вычитание статических дрейфов
		for (i = 0; i < 3; i++) {
			df0[i]  = df0_app[3*i  ];
			df0[i] += df0_app[3*i+1]*fsnav->imu->Tf[i];
			df0[i] += df0_app[3*i+2]*fsnav->imu->Tf[i]*fsnav->imu->Tf[i];

			fsnav->imu->f[i] -= df0[i];
		}
			// вычитание ошибки, связанной с перекосами и масштабами
		fsnav_linal_mul_u(&df[0], fsnav->imu->f, &Gamma[0], 1, 3);
		for (i = 0; i < 3; i++)
			fsnav->imu->f[i] -= df[i];
	}
}





// алгоритмы навигации
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
				диапазон: +0 до +inf
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
	static int n = 0;    // счетчик измерений

	char   *cfg_ptr; // указатель на параметр в строке конфигурации
	double  n1_n;    // (n-1)/n
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
	else if (fsnav->mode < 0) {}
	
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

	/*
		алгоритм выставки для датчиков низкого класса точности
		тангаж и крен вычисляются, курс равен нулю; матрица ориентации вычисляется по этим углам
		рекомендуется для систем, в которых измерения гироскопов не позволяют произвести полную выставку

		roll  = atan2( <f_2>, <f_3>)
		pitch = atan2(-<f_1>, sqrt(<f_2>^2+<f_3>^2)
		yaw   = 0
		здесь <f_i> — среднее значение показаний акселерометра на этапе выставки 
			
		использует:
			fsnav->imu->t
			fsnav->imu->f
			fsnav->imu->f_valid
		изменяет:
			fsnav->imu->sol.L
			fsnav->imu->sol.L_valid
			fsnav->imu->sol.q
			fsnav->imu->sol.q_valid
			fsnav->imu->sol.rpy
			fsnav->imu->sol.rpy_valid
			fsnav->imu->sol.v
			fsnav->imu->sol.v_valid
		параметры:
			{imu: alignment} — длительность выставки, сек
				тип: число с плавающей точкой
				диапазон: +0 до +inf
				по умолчанию: 300
				пример: {imu: alignment = 900}
	*/
void fsnav_ins_alignment_static_accs(void) {

	const char   t0_token[] = "alignment"; // параметр длительности выставки в конфигурационной строке
	const double t0_default = 300;         // стандартная длительность выставки

	static double 
		f[3] = {0,0,0}, // среднее значение показаний акселерометров
		t0      = -1;   // длительность выставки
	static int n = 0;   // счетчик количества измерений

	char   *cfg_ptr;    // указатель на параметр в строке конфигурации
	double  n1_n;       // (n-1)/n
	size_t  i;          // индекс

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
	else if (fsnav->mode < 0) {}

	// операции на каждом шаге
	else {
		// проверка времени выставки
		if (fsnav->imu->t > t0)
			return;
		// обнуление флагов достоверности
		fsnav->imu->sol.L_valid   = 0;
		fsnav->imu->sol.q_valid   = 0;
		fsnav->imu->sol.rpy_valid = 0;
		// обновление среднего и углов ориентации
		if (fsnav->imu->f_valid) {
			n++;
			n1_n = (n - 1.0)/n;
			for (i = 0; i < 3; i++)
				f[i] = f[i]*n1_n + fsnav->imu->f[i]/n;

			fsnav->imu->sol.rpy[0] = atan2(-f[2], f[1]);
			fsnav->imu->sol.rpy[1] = atan2( f[0], sqrt(f[1]*f[1] + f[2]*f[2]));
			fsnav->imu->sol.rpy[2] = 0.0;
		}
		fsnav->imu->sol.rpy_valid = 1;
		// обновление матрицы ориентации
		fsnav_linal_rpy2mat(fsnav->imu->sol.L, fsnav->imu->sol.rpy);
		fsnav->imu->sol.L_valid = 1;
		// обновление кватерниона
		fsnav_linal_mat2quat(fsnav->imu->sol.q  , fsnav->imu->sol.L);
		fsnav->imu->sol.q_valid = 1;
		// обнуление скорости (поскольку предполагается статическая выставка)
		for (i = 0; i < 3; i++)
			fsnav->imu->sol.v[i] = 0;
		fsnav->imu->sol.v_valid = 1;
	}
}

	/*
		установка априорных значений углов ориентации на выставке			
		использует:
			fsnav->imu->t
		изменяет:
			fsnav->imu->sol.L
			fsnav->imu->sol.L_valid
			fsnav->imu->sol.q
			fsnav->imu->sol.q_valid
			fsnav->imu->sol.rpy
			fsnav->imu->sol.rpy_valid
			fsnav->imu->sol.v
			fsnav->imu->sol.v_valid
		параметры:
			{imu: alignment} — длительность выставки, сек
				тип: число с плавающей точкой
				диапазон: +0 до +inf
				по умолчанию: 300
				пример: {imu: alignment = 900}
			{imu: roll, pitch, yaw} — углы ориентации, град
				тип: число с плавающей точкой
				диапазон: -inf до +inf
				по умолчанию: 0, 0, 0
				пример: {imu: roll = 0, pitch = 0, yaw = 135}
	*/
void fsnav_ins_alignment_static_const(void)
{
	size_t i;

	const char   t0_token[] = "alignment"; // параметр длительности выставки в конфигурационной строке
	const double t0_default = 300;         // стандартная длительность выставки

	const char*  rpy_tokens[]   = {"roll", "pitch", "yaw"};
	const double rpy_defaults[] = {0.0, 0.0, 0.0};
	
	static double t0      = -1; // длительность выставки
	static double rpy[3];

	char   *cfg_ptr; // указатель на параметр в строке конфигурации

	// проверка инерциальной подсистемы на шине
	if (fsnav->imu == NULL)
		return;

	// инициализация
	if (fsnav->mode == 0) {
		// парсинг длительности выставки в конфигурационной строке
		cfg_ptr = fsnav_locate_token(t0_token, fsnav->imu->cfg, fsnav->imu->cfglength, '=');
		if (cfg_ptr != NULL)
			t0 = atof(cfg_ptr);		
		if (cfg_ptr == NULL || t0 <= 0)
			t0 = t0_default;

		// парсинг значений углов ориентации
		for (i = 0; i < 3; i++) {
			cfg_ptr = fsnav_locate_token(rpy_tokens[i], fsnav->imu->cfg, fsnav->imu->cfglength, '=');
			if (cfg_ptr != NULL) {
				rpy[i] = atof(cfg_ptr);
				rpy[i] /= fsnav->imu_const.rad2deg;
			}
			else
				rpy[i] = rpy_defaults[i];
		}
	}

	// завершение работы
	else if (fsnav->mode < 0) {}

	// операции на каждом шаге
	else {
		// проверка времени выставки
		if (fsnav->imu->t > t0)
			return;
		// обнуление флагов достоверности
		fsnav->imu->sol.L_valid   = 0;
		fsnav->imu->sol.q_valid   = 0;
		fsnav->imu->sol.rpy_valid = 0;
		// присвоение значений углов ориентации
		for (i = 0; i < 3; i++)
			fsnav->imu->sol.rpy[i] = rpy[i];
		fsnav->imu->sol.rpy_valid = 1;
		// обновление матрицы ориентации
		fsnav_linal_rpy2mat(fsnav->imu->sol.L, fsnav->imu->sol.rpy);
		fsnav->imu->sol.L_valid = 1;
		// обновление кватерниона
		fsnav_linal_mat2quat(fsnav->imu->sol.q  , fsnav->imu->sol.L);
		fsnav->imu->sol.q_valid = 1;
		// обнуление скорости (поскольку предполагается статическая выставка)
		for (i = 0; i < 3; i++)
			fsnav->imu->sol.v[i] = 0;
		fsnav->imu->sol.v_valid = 1;
	}
}

	/*
		обнуление угла курса
		использует:
			не использует данные шины
		изменяет:
			fsnav->imu->sol.rpy_valid
			fsnav->imu->sol.L_valid
			fsnav->imu->sol.rpy[2]
			fsnav->imu->sol.L
		параметры:
			не использует параметров
	*/
void fsnav_ins_set_yaw_zero(void)
{
	// проверка инерциальной подсистемы на шине
	if (fsnav->imu == NULL)
		return;

	// инициализация
	if (fsnav->mode == 0) {}

	// завершение работы
	else if (fsnav->mode < 0) {}

	// операции на каждом шаге
	else {
		// сброс флагов достоверности
		fsnav->imu->sol.rpy_valid = 0;
		fsnav->imu->sol.  L_valid = 0;

		// обнуление угла курса
		fsnav->imu->sol.rpy[2] = 0.0;
		fsnav_linal_rpy2mat(fsnav->imu->sol.L, fsnav->imu->sol.rpy);

		// установка флагов достоверности
		fsnav->imu->sol.rpy_valid = 1;
		fsnav->imu->sol.  L_valid = 1;
	}
}

	/*
		фильтр Маджвика для счисления ориентации
		использует:
			fsnav->imu->t
			fsnav->imu->f
			fsnav->imu->f_valid
			fsnav->imu->w
			fsnav->imu->w_valid
			fsnav->imu->sol.L
			fsnav->imu->sol.L_valid
		изменяет:
			fsnav->imu->sol.q
			fsnav->imu->sol.q_valid
			fsnav->imu->sol.rpy
			fsnav->imu->sol.rpy_valid
		параметры:
			{imu: madgwick_feedback_rate} — параметр настройки фильтра Маджвика, градусы/сек
				тип: число с плавающей точкой
				диапазон: +0 до +inf
				пример: {imu: madgwick_feedback_rate = 0.003}
	*/
void fsnav_ins_attitude_madgwick(void) {

	static double t0 = -1;  // начальное время
	static double
		C[9],               // переходная матрица
		*L;                 // указатель на матрицу ориентации навигационного решения

	char *cfg_ptr;                      // указатель на параметр в строке конфигурации
	const char madgwick_token[] = "madgwick_feedback_rate";
	static double feedback_rate = -1;   // настраиваемый параметр, рад/сек, ~sqrt(3)*(остаточный дрейф гироскопов)
	const double epsilon = 1.0/1048576; // 2^-20 ~ 1e-6 

	double
		a[3],
		s,
		dt,                         // шаг времени
		w_madgwick[4],              // угловая скорость
		a_madgwick[3],              // измерения акселерометров
		J_madgwick[3][4],           // якобиан
		f_madgwick[3],              // целевая функция
		vector1_4[4],               // grad(f)
		vector2_4[4];               // 2*dq/dt
	size_t i;                       // индекс

	// инициализация
	if (fsnav->mode == 0) {

		// проверка инерциальной подсистемы на шине
		if (fsnav->imu == NULL)
			return;
		// сброс флагов достоверности
		fsnav->imu->sol.q_valid = 0;
		fsnav->imu->sol.L_valid = 0;
		fsnav->imu->sol.rpy_valid = 0;
		// указатель на матрицу ориентации imu->sol
		L = fsnav->imu->sol.L;
		// единичный кватернион
		for (i = 1, fsnav->imu->sol.q[0] = 1; i < 4; i++)
			fsnav->imu->sol.q[i] = 0;
		fsnav->imu->sol.q_valid = 1;
		// матрица ориентации
		fsnav_linal_quat2mat(L, fsnav->imu->sol.q);
		fsnav->imu->sol.L_valid = 1;
		// углы ориентации, соответствующие матрице ориентации
		fsnav_linal_mat2rpy(fsnav->imu->sol.rpy, L);
		fsnav->imu->sol.rpy_valid = 1;
		// обнуление начального времени
		t0 = -1;

		cfg_ptr = fsnav_locate_token(madgwick_token, fsnav->imu->cfg, fsnav->imu->cfglength, '=');
		if (cfg_ptr != NULL)
			feedback_rate = atof(cfg_ptr)/fsnav->imu_const.rad2deg;
		else
			return;

	}

	// завершение работы
	else if (fsnav->mode < 0) {}
	
	// операции на каждом шаге
	else {
		// проверка достоверности необходимых данных
		if (!fsnav->imu->sol.L_valid || !fsnav->imu->w_valid || !fsnav->imu->f_valid)
			return;
		// временнЫе переменные
		if (t0 < 0) {
			t0 = fsnav->imu->t;
			return;
		}
		dt = fsnav->imu->t - t0;
		t0 = fsnav->imu->t;

		for (i = 0, w_madgwick[0] = 0; i < 3; i++)
			w_madgwick[i+1] = fsnav->imu->w[i];

		for (i = 0, w_madgwick[0] = 0; i < 3; i++)
			a_madgwick[i] = fsnav->imu->f[i];

		double g0 = 0;
		double g1 = 0;
		double g2 = 1.0;
		// формирование целевой функции
		s = fsnav_linal_vnorm(a_madgwick, 3);
		f_madgwick[0] = 2.0*g0*(0.5 - fsnav->imu->sol.q[2]*fsnav->imu->sol.q[2] - fsnav->imu->sol.q[3]*fsnav->imu->sol.q[3])
		              + 2.0*g1*(      fsnav->imu->sol.q[0]*fsnav->imu->sol.q[3] + fsnav->imu->sol.q[1]*fsnav->imu->sol.q[2])
		              + 2.0*g2*(      fsnav->imu->sol.q[1]*fsnav->imu->sol.q[3] - fsnav->imu->sol.q[0]*fsnav->imu->sol.q[2]) - a_madgwick[0]/s;
		f_madgwick[1] = 2.0*g0*(      fsnav->imu->sol.q[1]*fsnav->imu->sol.q[2] - fsnav->imu->sol.q[0]*fsnav->imu->sol.q[3])
		              + 2.0*g1*(0.5 - fsnav->imu->sol.q[1]*fsnav->imu->sol.q[1] - fsnav->imu->sol.q[3]*fsnav->imu->sol.q[3])
		              + 2.0*g2*(      fsnav->imu->sol.q[0]*fsnav->imu->sol.q[1] + fsnav->imu->sol.q[2]*fsnav->imu->sol.q[3]) - a_madgwick[1]/s;
		f_madgwick[2] = 2.0*g0*(      fsnav->imu->sol.q[0]*fsnav->imu->sol.q[2] + fsnav->imu->sol.q[1]*fsnav->imu->sol.q[3])
		              + 2.0*g1*(      fsnav->imu->sol.q[2]*fsnav->imu->sol.q[3] - fsnav->imu->sol.q[0]*fsnav->imu->sol.q[1])
		              + 2.0*g2*(0.5 - fsnav->imu->sol.q[1]*fsnav->imu->sol.q[1] - fsnav->imu->sol.q[2]*fsnav->imu->sol.q[2]) - a_madgwick[2]/s;
		// формирование якобиана
		J_madgwick[0][0] =  2.0*g1*fsnav->imu->sol.q[3] - 2.0*g2*fsnav->imu->sol.q[2]; 
		J_madgwick[0][1] =  2.0*g1*fsnav->imu->sol.q[2] + 2.0*g2*fsnav->imu->sol.q[3]; 
		J_madgwick[0][2] = -4.0*g0*fsnav->imu->sol.q[2] + 2.0*g1*fsnav->imu->sol.q[1] - 2.0*g2*fsnav->imu->sol.q[0]; 
		J_madgwick[0][3] = -4.0*g0*fsnav->imu->sol.q[3] + 2.0*g1*fsnav->imu->sol.q[0] + 2.0*g2*fsnav->imu->sol.q[1];
		J_madgwick[1][0] = -2.0*g0*fsnav->imu->sol.q[3] + 2.0*g2*fsnav->imu->sol.q[1];
		J_madgwick[1][1] =  2.0*g0*fsnav->imu->sol.q[2] - 4.0*g1*fsnav->imu->sol.q[1] + 2.0*g2*fsnav->imu->sol.q[0];
		J_madgwick[1][2] =  2.0*g0*fsnav->imu->sol.q[1] + 2.0*g2*fsnav->imu->sol.q[3];
		J_madgwick[1][3] = -2.0*g0*fsnav->imu->sol.q[0] - 4.0*g1*fsnav->imu->sol.q[3] + 2.0*g2*fsnav->imu->sol.q[2];
		J_madgwick[2][0] =  2.0*g0*fsnav->imu->sol.q[2] - 2.0*g1*fsnav->imu->sol.q[1];
		J_madgwick[2][1] =  2.0*g0*fsnav->imu->sol.q[3] - 2.0*g1*fsnav->imu->sol.q[0] - 4.0*g2*fsnav->imu->sol.q[1];
		J_madgwick[2][2] =  2.0*g0*fsnav->imu->sol.q[0] + 2.0*g1*fsnav->imu->sol.q[3] - 4.0*g2*fsnav->imu->sol.q[2];
		J_madgwick[2][3] =  2.0*g0*fsnav->imu->sol.q[1] + 2.0*g1*fsnav->imu->sol.q[2];
		
		for (i = 0; i < 4; i++)
			vector1_4[i] = J_madgwick[0][i]*f_madgwick[0] + J_madgwick[1][i]*f_madgwick[1] + J_madgwick[2][i]*f_madgwick[2];

		fsnav_linal_qmul(vector2_4, fsnav->imu->sol.q, w_madgwick);
		// интегрирование
		for (i = 0; i < 4; i++) // для избежания деления на ноль
			fsnav->imu->sol.q[i] += 0.5*(vector2_4[i] - feedback_rate*vector1_4[i]/(epsilon + fsnav_linal_vnorm(vector1_4,4)))*dt;

		// норма
		s = fsnav_linal_vnorm(fsnav->imu->sol.q, 4);
		for (i = 0; i < 4; i++)
			fsnav->imu->sol.q[i] /= s;

		// L = L(q)
		fsnav_linal_quat2mat(L, fsnav->imu->sol.q);

		// опорный трёхгранник
		// a <- c = (W + u)*dt
		for (i = 0; i < 3; i++)
			a[i] = fsnav->imu->W_valid ? fsnav->imu->W[i] : 0;
		if (fsnav->imu->sol.llh_valid) {
			a[1] += fsnav->imu_const.u*cos(fsnav->imu->sol.llh[1]);
			a[2] += fsnav->imu_const.u*sin(fsnav->imu->sol.llh[1]);
		}
		for (i = 0; i < 3; i++)
			a[i] *= dt;
		// L = L*(E + [c x]*sin(c)/|c| + [c x]^2*(1-cos(c))/|c|^2)^T
		fsnav_linal_eul2mat(C, a);	// C = E + [c x]*sin(c)/|c| + [c x]^2*(1-cos(c))/|c|^2
		for (i = 0; i < 9; i += 3) { // L = L*C^T
			a[0] = L[i+0];
			a[1] = L[i+1]; 
			a[2] = L[i+2];
			L[i+0] = a[0]*C[0] + a[1]*C[1] + a[2]*C[2];
			L[i+1] = a[0]*C[3] + a[1]*C[4] + a[2]*C[5];
			L[i+2] = a[0]*C[6] + a[1]*C[7] + a[2]*C[8];
		}
		// обновление кватерниона ориентации
		fsnav_linal_mat2quat(fsnav->imu->sol.q, L);
		fsnav->imu->sol.q_valid = 1;
		// обновление углов ориентации
		fsnav_linal_mat2rpy(fsnav->imu->sol.rpy, L);
		fsnav->imu->sol.rpy_valid = 1;
	}
}