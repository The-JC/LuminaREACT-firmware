target_sources(${elf_file}
	PRIVATE
		main.c
		color.c
		ws2812.c
)

add_subdirectory(drivers)
