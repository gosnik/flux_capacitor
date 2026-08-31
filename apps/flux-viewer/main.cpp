#include "viewer_app.hpp"

#include <SDL.h>
#include <SDL_opengl.h>
#include <backends/imgui_impl_opengl3.h>
#include <backends/imgui_impl_sdl2.h>
#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace {

constexpr float default_ui_scale = 1.0F;
constexpr float minimum_ui_scale = 0.75F;
constexpr float maximum_ui_scale = 2.0F;
constexpr float ui_scale_step = 0.05F;

void print_usage(const char* program) {
  std::cout << "Usage: " << program << " [options]\n"
            << "  --connect <ws://host:port/telemetry>  Live endpoint (default localhost:8765)\n"
            << "  --no-connect                          Start disconnected\n"
            << "  --level <normal|diagnostic|trace>     Requested detail (default normal)\n"
            << "  --rate <Hz>                           Snapshot/event batch rate (1-60)\n"
            << "  --replay <trace.jsonl>                Open a recorded telemetry trace\n"
            << "  --record <trace.jsonl>                Record incoming live events\n"
            << "  --help                                Show this help\n";
}

flux::viewer::LaunchOptions parse_options(int argc, char** argv) {
  flux::viewer::LaunchOptions options;
  for (int index = 1; index < argc; ++index) {
    const std::string_view option = argv[index];
    const auto value = [&](const char* name) -> std::string {
      if (index + 1 >= argc) {
        throw std::runtime_error(std::string(name) + " requires a value");
      }
      ++index;
      return argv[index];
    };
    if (option == "--connect") {
      options.endpoint = value("--connect");
      options.connect_on_start = true;
    } else if (option == "--no-connect") {
      options.connect_on_start = false;
    } else if (option == "--level") {
      const std::string level = value("--level");
      if (level != "normal" && level != "diagnostic" && level != "trace") {
        throw std::runtime_error("--level must be normal, diagnostic, or trace");
      }
      options.level = flux::telemetry::level_from_string(level);
    } else if (option == "--rate") {
      options.rate_hz = std::stod(value("--rate"));
      if (options.rate_hz < 1.0 || options.rate_hz > 60.0) {
        throw std::runtime_error("--rate must be between 1 and 60 Hz");
      }
    } else if (option == "--replay") {
      options.replay_path = value("--replay");
      options.connect_on_start = false;
    } else if (option == "--record") {
      options.record_path = value("--record");
    } else if (option == "--help" || option == "-h") {
      print_usage(argv[0]);
      std::exit(0);
    } else {
      throw std::runtime_error("unknown option: " + std::string(option));
    }
  }
  return options;
}

void apply_flux_style(float ui_scale) {
  ImGuiStyle& style = ImGui::GetStyle();
  style = ImGuiStyle{};
  ImGui::StyleColorsDark(&style);
  style.WindowRounding = 4.0F;
  style.ChildRounding = 3.0F;
  style.FrameRounding = 3.0F;
  style.ScrollbarRounding = 3.0F;
  style.WindowPadding = ImVec2(9.0F, 8.0F);
  style.FramePadding = ImVec2(7.0F, 4.0F);
  style.ItemSpacing = ImVec2(7.0F, 6.0F);
  style.Colors[ImGuiCol_WindowBg] = ImVec4(0.045F, 0.065F, 0.083F, 1.0F);
  style.Colors[ImGuiCol_ChildBg] = ImVec4(0.050F, 0.071F, 0.090F, 1.0F);
  style.Colors[ImGuiCol_Header] = ImVec4(0.12F, 0.31F, 0.43F, 0.80F);
  style.Colors[ImGuiCol_HeaderHovered] = ImVec4(0.16F, 0.42F, 0.58F, 0.85F);
  style.Colors[ImGuiCol_Button] = ImVec4(0.11F, 0.30F, 0.42F, 0.90F);
  style.Colors[ImGuiCol_ButtonHovered] = ImVec4(0.16F, 0.43F, 0.58F, 1.0F);
  style.Colors[ImGuiCol_CheckMark] = ImVec4(0.23F, 0.83F, 0.84F, 1.0F);
  style.Colors[ImGuiCol_SliderGrab] = ImVec4(0.25F, 0.76F, 0.84F, 1.0F);
  style.ScaleAllSizes(ui_scale);
#if IMGUI_VERSION_NUM >= 19200
  style.FontScaleMain = ui_scale;
#else
  ImGui::GetIO().FontGlobalScale = ui_scale;
#endif
}

void set_ui_scale(float& current_scale, float requested_scale) {
  const float clamped = std::clamp(requested_scale, minimum_ui_scale, maximum_ui_scale);
  if (std::abs(clamped - current_scale) < 0.001F) {
    return;
  }
  current_scale = clamped;
  apply_flux_style(current_scale);
}

bool process_ui_scale_event(const SDL_Event& event, float& ui_scale) {
  if (event.type == SDL_MOUSEWHEEL) {
    const Uint16 key_mod = static_cast<Uint16>(SDL_GetModState());
    const Uint16 scale_modifiers = static_cast<Uint16>(KMOD_CTRL | KMOD_GUI);
    if ((key_mod & scale_modifiers) != 0U) {
#if SDL_VERSION_ATLEAST(2, 0, 18)
      float wheel = event.wheel.preciseY;
      if (wheel == 0.0F) {
        wheel = static_cast<float>(event.wheel.y);
      }
#else
      const float wheel = static_cast<float>(event.wheel.y);
#endif
      if (wheel != 0.0F) {
        set_ui_scale(ui_scale, ui_scale + (wheel * ui_scale_step));
        return true;
      }
    }
  }

  if (event.type == SDL_KEYDOWN) {
    const Uint16 key_mod = event.key.keysym.mod;
    const Uint16 scale_modifiers = static_cast<Uint16>(KMOD_CTRL | KMOD_GUI);
    if ((key_mod & scale_modifiers) != 0U) {
      switch (event.key.keysym.sym) {
      case SDLK_EQUALS:
      case SDLK_PLUS:
      case SDLK_KP_PLUS:
      case SDLK_KP_EQUALS:
        set_ui_scale(ui_scale, ui_scale + ui_scale_step);
        return true;
      case SDLK_MINUS:
      case SDLK_KP_MINUS:
      case SDLK_UNDERSCORE:
        set_ui_scale(ui_scale, ui_scale - ui_scale_step);
        return true;
      case SDLK_0:
      case SDLK_KP_0:
        set_ui_scale(ui_scale, default_ui_scale);
        return true;
      default:
        break;
      }
    }
  }

  return false;
}

} // namespace

int main(int argc, char** argv) {
  flux::viewer::LaunchOptions options;
  try {
    options = parse_options(argc, argv);
  } catch (const std::exception& exception) {
    std::cerr << "flux-viewer: " << exception.what() << "\n\n";
    print_usage(argv[0]);
    return 2;
  }

  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
    std::cerr << "flux-viewer: SDL initialization failed: " << SDL_GetError() << '\n';
    return 1;
  }

#if defined(__APPLE__)
  constexpr const char* glsl_version = "#version 150";
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_FORWARD_COMPATIBLE_FLAG);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
#else
  constexpr const char* glsl_version = "#version 130";
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
#endif
  SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
  SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 0);
  SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 0);

  SDL_Window* window = SDL_CreateWindow(
      "Flux Router Telemetry Viewer", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1500, 920,
      SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
  if (window == nullptr) {
    std::cerr << "flux-viewer: window creation failed: " << SDL_GetError() << '\n';
    SDL_Quit();
    return 1;
  }
  SDL_GLContext context = SDL_GL_CreateContext(window);
  if (context == nullptr) {
    std::cerr << "flux-viewer: OpenGL context creation failed: " << SDL_GetError() << '\n';
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 1;
  }
  SDL_GL_MakeCurrent(window, context);
  SDL_GL_SetSwapInterval(1);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  std::string imgui_ini_path;
  if (char* preferences = SDL_GetPrefPath("Flux", "RouterTelemetryViewer")) {
    imgui_ini_path = std::string(preferences) + "imgui.ini";
    SDL_free(preferences);
  }
  io.IniFilename = imgui_ini_path.empty() ? nullptr : imgui_ini_path.c_str();
  apply_flux_style(default_ui_scale);
  ImGui_ImplSDL2_InitForOpenGL(window, context);
  ImGui_ImplOpenGL3_Init(glsl_version);

  int result = 0;
  try {
    flux::viewer::ViewerApp app(std::move(options));
    bool running = true;
    float ui_scale = default_ui_scale;
    while (running && !app.wants_exit()) {
      SDL_Event event;
      while (SDL_PollEvent(&event) != 0) {
        if (!process_ui_scale_event(event, ui_scale)) {
          ImGui_ImplSDL2_ProcessEvent(&event);
        }
        if (event.type == SDL_QUIT) {
          running = false;
        }
      }

      app.tick();
      ImGui_ImplOpenGL3_NewFrame();
      ImGui_ImplSDL2_NewFrame();
      ImGui::NewFrame();
      app.draw();
      ImGui::Render();

      int drawable_width = 0;
      int drawable_height = 0;
      SDL_GL_GetDrawableSize(window, &drawable_width, &drawable_height);
      glViewport(0, 0, drawable_width, drawable_height);
      glClearColor(0.025F, 0.038F, 0.052F, 1.0F);
      glClear(GL_COLOR_BUFFER_BIT);
      ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
      SDL_GL_SwapWindow(window);
    }
  } catch (const std::exception& exception) {
    std::cerr << "flux-viewer: " << exception.what() << '\n';
    result = 1;
  }

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplSDL2_Shutdown();
  ImGui::DestroyContext();
  SDL_GL_DeleteContext(context);
  SDL_DestroyWindow(window);
  SDL_Quit();
  return result;
}
