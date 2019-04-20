use kiss3d::window::Window;
use kiss3d::conrod::{self, Ui, Widget, Positionable, Sizeable, Colorable, Labelable, Borderable};
use nphysics::world::World;

use crate::testbed::{TestbedState, RunMode, TestbedStateFlags, TestbedActionFlags};


const SIDEBAR_W: f64 = 200.0;
const ELEMENT_W: f64 = SIDEBAR_W - 20.0;
const ELEMENT_H: f64 = 20.0;
const VSPACE: f64 = 4.0;
const TITLE_VSPACE: f64 = 4.0;
const LEFT_MARGIN: f64 = 10.0;
const ALPHA: f32 = 0.9;

widget_ids! {
    pub struct ConrodIds {
        canvas,
        title_demos_list,
        title_slider_vel_iter,
        title_slider_pos_iter,
        title_warmstart_coeff,
        title_frequency,
        demos_list,
        button_pause,
        button_single_step,
        button_restart,
        button_quit,
        slider_vel_iter,
        slider_pos_iter,
        slider_warmstart_coeff,
        slider_frequency,
        toggle_sleep,
        toggle_warm_starting,
        toggle_time_of_impact,
        toggle_sub_stepping,
        toggle_shapes,
        toggle_joints,
        toggle_aabbs,
        toggle_contact_points,
        toggle_contact_normals,
        toggle_center_of_masses,
        toggle_statistics,
        toggle_profile,
        toggle_wireframe,
        separator1,
        separator2,
    }
}



pub struct TestbedUi {
    ids: ConrodIds
}

impl TestbedUi {
    pub fn new(window: &mut Window) -> Self {
        use conrod::position::{Align, Direction, Padding, Position, Relative};

        let mut ui = window.conrod_ui_mut();
        ui.theme = conrod::Theme {
            name: "Testbed theme".to_string(),
            padding: Padding::none(),
            x_position: Position::Relative(Relative::Align(Align::Start), None),
            y_position: Position::Relative(Relative::Direction(Direction::Backwards, 20.0), None),
            background_color: conrod::color::DARK_CHARCOAL.alpha(ALPHA),
            shape_color: conrod::color::LIGHT_CHARCOAL.alpha(ALPHA),
            border_color: conrod::color::BLACK.alpha(ALPHA),
            border_width: 0.0,
            label_color: conrod::color::WHITE.alpha(ALPHA),
            font_id: None,
            font_size_large: 15,
            font_size_medium: 11,
            font_size_small: 8,
            widget_styling: conrod::theme::StyleMap::default(),
            mouse_drag_threshold: 0.0,
            double_click_threshold: std::time::Duration::from_millis(500),
        };

        Self {
            ids: ConrodIds::new(ui.widget_id_generator())
        }
    }

    pub fn update(&mut self, window: &mut Window, world: &mut World<f32>, state: &mut TestbedState) {
        let ui_root = window.conrod_ui().window;
        let mut ui = window.conrod_ui_mut().set_widgets();
        conrod::widget::Canvas::new()
//            .title_bar("Demos")
//            .title_bar_color(conrod::color::Color::Rgba(1.0, 0.0, 0.0, 1.0))
//            .pad(100.0)
//            .pad_left(MARGIN)
//            .pad_right(MARGIN)
            .scroll_kids_vertically()
            .mid_right_with_margin(10.0)
            .w(SIDEBAR_W)
            .padded_h_of(ui_root, 10.0)
            .set(self.ids.canvas, &mut ui);

        conrod::widget::Text::new("Select example:")
            .top_left_with_margins_on(self.ids.canvas, VSPACE, LEFT_MARGIN)
//            .w_h(ELEMENT_W, ELEMENT_H)
            .set(self.ids.title_demos_list, &mut ui);

        for selected in conrod::widget::DropDownList::new(&state.example_names, Some(state.selected_example))
//            .mid_top_with_margin_on(self.ids.canvas, 20.0)
            .align_middle_x_of(self.ids.canvas)
            .down_from(self.ids.title_demos_list, TITLE_VSPACE)
            .left_justify_label()
            .w_h(ELEMENT_W, ELEMENT_H)
            .color(conrod::color::LIGHT_CHARCOAL)
            .set(self.ids.demos_list, &mut ui) {
            if selected != state.selected_example {
                state.selected_example = selected;
                state.action_flags.set(TestbedActionFlags::EXAMPLE_CHANGED, true)
            }
        }

        separator(self.ids.canvas, self.ids.demos_list, self.ids.separator1, &mut ui);


        let curr_vel_iters = world.integration_parameters().max_velocity_iterations;
        let curr_pos_iters = world.integration_parameters().max_position_iterations;
        let curr_warmstart_coeff = world.integration_parameters().warmstart_coeff;
        let curr_frequency = (1.0 / world.integration_parameters().dt).round() as usize;


        conrod::widget::Text::new("Vel. Iters.:")
            .down_from(self.ids.separator1, VSPACE)
            .set(self.ids.title_slider_vel_iter, &mut ui);

        for val in conrod::widget::Slider::new(curr_vel_iters as f32, 0.0, 50.0)
            .label(&curr_vel_iters.to_string())
            .align_middle_x_of(self.ids.canvas)
            .down_from(self.ids.title_slider_vel_iter, TITLE_VSPACE)
            .w_h(ELEMENT_W, ELEMENT_H)
            .set(self.ids.slider_vel_iter, &mut ui) {
            world.integration_parameters_mut().max_velocity_iterations = val as usize;
        }


        conrod::widget::Text::new("Pos. Iters.:")
            .down_from(self.ids.slider_vel_iter, VSPACE)
            .set(self.ids.title_slider_pos_iter, &mut ui);

        for val in conrod::widget::Slider::new(curr_pos_iters as f32, 0.0, 50.0)
            .label(&curr_pos_iters.to_string())
            .align_middle_x_of(self.ids.canvas)
            .down_from(self.ids.title_slider_pos_iter, TITLE_VSPACE)
            .w_h(ELEMENT_W, ELEMENT_H)
            .set(self.ids.slider_pos_iter, &mut ui) {
            world.integration_parameters_mut().max_position_iterations = val as usize;
        }



        conrod::widget::Text::new("Warm-start coeff.:")
            .down_from(self.ids.slider_pos_iter, VSPACE)
            .set(self.ids.title_warmstart_coeff, &mut ui);

        for val in conrod::widget::Slider::new(curr_warmstart_coeff as f32, 0.0, 1.0)
            .label(&format!("{:.2}", curr_warmstart_coeff))
            .align_middle_x_of(self.ids.canvas)
            .down_from(self.ids.title_warmstart_coeff, TITLE_VSPACE)
            .w_h(ELEMENT_W, ELEMENT_H)
            .set(self.ids.slider_warmstart_coeff, &mut ui) {
            world.integration_parameters_mut().warmstart_coeff = val;
        }


        conrod::widget::Text::new("Frequency:")
            .down_from(self.ids.slider_warmstart_coeff, VSPACE)
            .set(self.ids.title_frequency, &mut ui);

        for val in conrod::widget::Slider::new(curr_frequency as f32, 1.0, 120.0)
            .label(&format!("{:.2}Hz", curr_frequency))
            .align_middle_x_of(self.ids.canvas)
            .down_from(self.ids.title_frequency, TITLE_VSPACE)
            .w_h(ELEMENT_W, ELEMENT_H)
            .set(self.ids.slider_frequency, &mut ui) {
            world.integration_parameters_mut().dt = 1.0 / val.round();
        }

        let toggle_list = [
            ("Sleep", self.ids.toggle_sleep, TestbedStateFlags::SLEEP),
//            ("Warm Starting", self.ids.toggle_warm_starting, TestbedStateFlags::WARM_STARTING),
//            ("Time of Impact", self.ids.toggle_time_of_impact, TestbedStateFlags::TIME_OF_IMPACT),
//            ("Sub-Stepping", self.ids.toggle_sub_stepping, TestbedStateFlags::SUB_STEPPING),
            ("", self.ids.separator2, TestbedStateFlags::NONE),
//            ("Shapes", self.ids.toggle_shapes, TestbedStateFlags::SHAPES),
            ("Joints", self.ids.toggle_joints, TestbedStateFlags::JOINTS),
            ("AABBs", self.ids.toggle_aabbs, TestbedStateFlags::AABBS),
            ("Contacts", self.ids.toggle_contact_points, TestbedStateFlags::CONTACT_POINTS),
//            ("Contact Normals", self.ids.toggle_contact_normals, TestbedStateFlags::CONTACT_NORMALS),
            ("Wireframe", self.ids.toggle_wireframe, TestbedStateFlags::WIREFRAME),
//            ("Center of Masses", self.ids.toggle_center_of_masses, TestbedStateFlags::CENTER_OF_MASSES),
            ("Statistics", self.ids.toggle_statistics, TestbedStateFlags::STATISTICS),
            ("Profile", self.ids.toggle_profile, TestbedStateFlags::PROFILE),
        ];

        toggles(&toggle_list, self.ids.canvas, self.ids.slider_frequency, &mut ui, &mut state.flags);

        for _press in conrod::widget::Button::new()
            .label("Pause (T)")
            .align_middle_x_of(self.ids.canvas)
            .down_from(self.ids.toggle_profile, VSPACE)
            .w_h(ELEMENT_W, ELEMENT_H)
            .set(self.ids.button_pause, &mut ui) {
            if state.running == RunMode::Stop {
                state.running = RunMode::Running
            } else {
                state.running = RunMode::Stop
            }
        }

        for _press in conrod::widget::Button::new()
            .label("Single Step (S)")
            .align_middle_x_of(self.ids.canvas)
            .down_from(self.ids.button_pause, VSPACE)
            .set(self.ids.button_single_step, &mut ui) {
            state.running = RunMode::Step
        }

        for _press in conrod::widget::Button::new()
            .label("Restart (R)")
            .align_middle_x_of(self.ids.canvas)
            .down_from(self.ids.button_single_step, VSPACE)
            .set(self.ids.button_restart, &mut ui) {
            state.action_flags.set(TestbedActionFlags::EXAMPLE_CHANGED, true);
        }

        #[cfg(not(target_arch = "wasm32"))]
        for _press in conrod::widget::Button::new()
            .label("Quit (ESC)")
            .align_middle_x_of(self.ids.canvas)
            .down_from(self.ids.button_restart, VSPACE)
            .set(self.ids.button_quit, &mut ui) {
            state.running = RunMode::Quit
        }
    }
}

fn toggles(toggles: &[(&str, conrod::widget::Id, TestbedStateFlags)], canvas: conrod::widget::Id, prev: conrod::widget::Id, ui: &mut conrod::UiCell, flags: &mut TestbedStateFlags) {
    toggle(toggles[0].0, toggles[0].2, canvas, prev, toggles[0].1, ui, flags);

    for win in toggles.windows(2) {
        toggle(win[1].0, win[1].2, canvas, win[0].1, win[1].1, ui, flags)
    }
}

fn toggle(title: &str, flag: TestbedStateFlags, canvas: conrod::widget::Id, prev: conrod::widget::Id, curr: conrod::widget::Id, ui: &mut conrod::UiCell, flags: &mut TestbedStateFlags) {
    if title == "" {
        // This is a separator.
        separator(canvas, prev, curr, ui)
    } else {
        for _pressed in conrod::widget::Toggle::new(flags.contains(flag))
            .mid_left_with_margin_on(canvas, LEFT_MARGIN)
            .down_from(prev, VSPACE)
            .w_h(20.0 /*ELEMENT_W*/, ELEMENT_H)
            .label(title)
            .label_color(kiss3d::conrod::color::WHITE)
            .label_x(conrod::position::Relative::Direction(conrod::position::Direction::Forwards, 5.0))
            .border(2.0)
//            .border_color(kiss3d::conrod::color::WHITE)
            .set(curr, ui) {
            flags.toggle(flag)
        }
    }
}

fn separator(canvas: conrod::widget::Id, prev: conrod::widget::Id, curr: conrod::widget::Id, ui: &mut conrod::UiCell) {
    conrod::widget::Line::centred([-ELEMENT_W / 2.0, 0.0], [ELEMENT_W / 2.0, 0.0])
        .align_middle_x_of(canvas)
        .down_from(prev, VSPACE)
        .w(ELEMENT_W)
        .set(curr, ui);
}