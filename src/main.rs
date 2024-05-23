use anyhow::Result;
use kiddo::KdTree;
use kiddo::NearestNeighbour;
use kiddo::SquaredEuclidean;
use nannou::glam::Vec3Swizzles;
use nannou::prelude::*;
use std::fs::read_to_string;
use std::fs::File;
use std::io::LineWriter;
use std::io::Write;
use std::thread;
use std::time::Duration;
use std::time::Instant;

// Curve that is being grown
struct Curve {
    points: Vec<Vec3A>,
    accelerations: Vec<Vec3A>,
}

// Static collider that constraints growth 
struct Collider {
    points: Vec<Vec3A>,
}

struct Model {
    curves: Vec<Curve>,
    colliders: Vec<Collider>,
    ms_per_frame: Duration,
    bounding_box: Rect<f32>,
    total_points: usize,
    draw_changing_points: bool,
}

fn main() {
    nannou::app(model)
        .loop_mode(LoopMode::rate_fps(120.0))
        .event(event)
        .simple_window(view)
        //.fullscreen()
        .run();
}

fn model(_app: &App) -> Model {
    let mut model = read_pp_file_to_model("./geometry.pp").unwrap();
    subdivide_colliders(&mut model);
    model
}

fn event(app: &App, model: &mut Model, event: Event) {
    match event {
        Event::Update(_) => {
            let mut max_change = 0.0;
            let start = Instant::now();
            for _ in 0..100 {
                model.total_points = 0;
                subdivide(model);
                push_area(app, model);

                for curve in model.curves.iter_mut() {
                    model.total_points += curve.points.len();
                    (0..curve.points.len()).for_each(|i| {
                        let ac_length = curve.accelerations[i].length();
                        if ac_length > max_change {
                            max_change = ac_length;
                        }

                        curve.points[i] += curve.accelerations[i] * 0.25;
                        curve.accelerations[i] *= 0.8;
                    });
                }
                app.draw();
            }

            model.ms_per_frame = start.elapsed();

            if max_change < 0.1 {
                println!("Stable")
            } else {
                println!(
                    "Points: {:?} | ms/frame: {:06.2} | Change: {:.2}",
                    model.total_points,
                    model.ms_per_frame.ms(),
                    max_change
                );
            }
        }
        Event::WindowEvent { id: _id, simple } => match simple {
            Some(WindowEvent::KeyPressed(key)) => match key {
                Key::P => {
                    write_to_file(model).unwrap();
                    println!("Wrote file");
                    thread::sleep(Duration::from_millis(1000));
                }
                Key::Numpad1 => {
                    model.draw_changing_points = !model.draw_changing_points;
                }

                _ => {}
            },
            _ => {}
        },
        _ => {}
    }
}

fn view(app: &App, model: &Model, frame: Frame) {
    let draw = app.draw();
    let win_rect = app.window_rect();

    draw.rect()
        .w_h(win_rect.w(), win_rect.h())
        .rgba(0.06666, 0.0666, 0.07, 1.0);
    let xy = model.bounding_box.xy();
    let scale = (win_rect.w() / model.bounding_box.w()).min(win_rect.h() / model.bounding_box.h());

    let draw = draw
        .scale(scale)
        .translate(Vec3::from_slice(&[-xy.x, -xy.y, 0.0]));

    let hue_offset = 1.0 / model.curves.len() as f32;

    for (curve_index, curve) in model.curves.iter().enumerate() {
        for (i, point) in curve.points.iter().enumerate() {
            if model.draw_changing_points {
                let acl = curve.accelerations[i].length();
                if acl > 0.1 {
                    draw.ellipse()
                        .w_h(8.0, 8.0)
                        .x_y(point.x, point.y)
                        .hsva(0.0, 1.0, 1.0, acl / 1.1 * 10.0);
                }
            }
            draw.line()
                .start(point.xy())
                .end(curve.points[(i + 1) % curve.points.len()].xy())
                .weight(2.0)
                .hsl(hue_offset * curve_index as f32, 0.7, 0.7);
        }
    }

    for collider in &model.colliders {
        for (i, point) in collider.points.iter().enumerate() {
            draw.line()
                .start(point.xy())
                .end(collider.points[(i + 1) % collider.points.len()].xy())
                .weight(2.0)
                .color(RED);
        }
    }

    let xy: Vec2 = app.window_rect().pad_left(60.0).pad_top(20.0).top_left();
    draw.rect()
        .w_h(200.0, 100.0)
        .xy(xy)
        .rgba(0.06666, 0.0666, 0.07, 1.0);
    draw.text(format!("{:}ms/frame", model.ms_per_frame.as_millis()).as_str())
        .xy(xy);

    draw.to_frame(app, &frame).unwrap();
}

// subdivide lines that are longer than SPLIT_DISTANCE
fn subdivide(model: &mut Model) {
    for curve in model.curves.iter_mut() {
        let mut to_insert = Vec::<(usize, Vec3A)>::new();
        for (index, current) in curve.points.iter().enumerate() {
            let next = curve.points[(index + 1) % curve.points.len()];

            let dir: Vec3A = next - *current;
            if dir.length() > SPLIT_DISTANCE {
                let new = *current + dir * 0.5;
                to_insert.push((index + 1 + to_insert.len(), new));
            }
        }

        for (index, point) in to_insert {
            curve.points.insert(index, point);
            curve.accelerations.insert(index, Vec3A::new(0.0, 0.0, 0.0));
        }
    }
}

// Subdivide colliders to be smaller than a fraction of the SPLIT_DISTANCE
// to ensure that curve points won't pass through
fn subdivide_colliders(model: &mut Model) {
    let mut stop = false;
    while !stop {
        for collider in model.colliders.iter_mut() {
            let mut to_insert = Vec::<(usize, Vec3A)>::new();
            for (index, current) in collider.points.iter().enumerate() {
                let next = collider.points[(index + 1) % collider.points.len()];
                let dir: Vec3A = next - *current;
                if dir.length() > SPLIT_DISTANCE * 0.3 {
                    let new = *current + dir * 0.5;
                    to_insert.push((index + 1 + to_insert.len(), new));
                }
            }

            stop = to_insert.is_empty();
            for (index, point) in to_insert {
                collider.points.insert(index, point);
            }
        }
    }
}





fn push_area(app: &App, model: &mut Model) {
    const STEP_SIZE: usize = 10; // Update every STEP_SIZE point per curve to speed up simulation

    // one tree per curve
    let mut trees = Vec::<KdTree<f32, 2>>::with_capacity(model.curves.len());
    for curve in &model.curves {
        let mut kdtree = KdTree::<f32, 2>::with_capacity(curve.points.len());
        for (index, current) in curve.points.iter().enumerate() {
            kdtree.add(&[current.x, current.y], index as u64);
        }
        trees.push(kdtree);
    }

    // one tree per collider
    let mut collider_trees = Vec::<KdTree<f32, 2>>::with_capacity(model.curves.len());
    for collider in &model.colliders {
        let mut kdtree = KdTree::<f32, 2>::with_capacity(collider.points.len());
        for (index, current) in collider.points.iter().enumerate() {
            kdtree.add(&[current.x, current.y], index as u64);
        }
        collider_trees.push(kdtree);
    }

    for current_curve_index in 0..model.curves.len() {
        for current_index in (app.elapsed_frames() as usize % STEP_SIZE
            ..model.curves[current_curve_index].points.len())
            .step_by(STEP_SIZE)
        {
            let current = &model.curves[current_curve_index].points[current_index];

            let mut force = Vec3A::new(0.0, 0.0, 0.0);

            for (tree_index, tree) in trees.iter().enumerate() {
                let nearest = tree.within_unsorted::<SquaredEuclidean>(
                    &[current.x, current.y],
                    SPLIT_DISTANCE * SPLIT_DISTANCE * 4.0,
                );

                for NearestNeighbour {
                    distance: dist,
                    item: index_neighbour,
                } in nearest
                {
                    if tree_index == current_curve_index
                        && current_index == index_neighbour as usize
                    {
                        continue;
                    }
                    let neighbour = model.curves[tree_index].points[index_neighbour as usize];
                    let dir: Vec3A = neighbour - *current;
                    force += dir.normalize() * (-1.0 / (dist).max(1.0));
                }
            }

            let points_len = model.curves[current_curve_index].points.len();

            let neighbour = &model.curves[current_curve_index].points
                [(current_index - 1 + points_len) % points_len];
            let mut dir = *neighbour - *current;
            let len = dir.length();
            dir = dir.normalize();
            if len < SPLIT_DISTANCE * 0.5 {
                dir *= -1.0;
            }
            force += dir * (1.0 / (len * len).max(1.0));

            if current_index + 1 < model.curves[current_curve_index].points.len() {
                let neighbour =
                    model.curves[current_curve_index].points[(current_index + 1) % points_len];
                let mut dir = neighbour - *current;
                let len = dir.length();
                dir = dir.normalize();
                if len < SPLIT_DISTANCE * 0.5 {
                    dir *= -1.0;
                }
                force += dir * (1.0 / (len * len).max(1.0));
            }

            for (tree_index, tree) in collider_trees.iter().enumerate() {
                let nearest =
                    tree.within_unsorted::<SquaredEuclidean>(&[current.x, current.y], 5.0);

                for NearestNeighbour {
                    distance: _dist,
                    item: index_neighbour,
                } in nearest
                {
                    let neighbour = model.colliders[tree_index].points[index_neighbour as usize];
                    let mut dir = neighbour - *current;
                    let len = dir.length();
                    dir = dir.normalize();
                    force += dir * (-1.0 / (len * len).max(1.0));
                }
            }

            model.curves[current_curve_index].accelerations[current_index] += force;
        }
    }
}

const SPLIT_DISTANCE: f32 = 8.0;

pub fn points_on_circle(
    origin_x: f32,
    origin_y: f32,
    radius: f32,
    amount_of_points: usize,
) -> Vec<Vec3A> {
    // https://www.mathopenref.com/coordcirclealgorithm.html

    let mut points: Vec<Vec3A> = Vec::new();

    let h: f32 = origin_x;
    let k: f32 = origin_y;

    let two_pi: f32 = 2.0 * PI;
    let step: f32 = two_pi / (amount_of_points as f32);
    let mut theta: f32 = 0.0;

    for _ in 0..amount_of_points {
        let x: f32 = h + radius * f32::cos(theta);
        let y: f32 = k + radius * f32::sin(theta);
        points.push(Vec3A::new(x, y, 0.0));
        theta += step;
    }

    points
}

fn read_pp_file_to_model(filename: &str) -> Result<Model> {
    let mut model = Model {
        curves: Vec::<Curve>::new(),
        colliders: Vec::<Collider>::new(),
        ms_per_frame: Duration::ZERO,
        bounding_box: Rect::from_w_h(0.0, 0.0),
        total_points: 0,
        draw_changing_points: false,
    };

    let mut points = Vec::<Vec3A>::new();
    let mut is_collider = false;
    let string = read_to_string(filename)?;
    let mut lines = string.lines().peekable();
    while let Some(line) = lines.next() {
        if line.starts_with('#') || lines.peek().is_none() {
            if !points.len().is_zero() {
                match is_collider {
                    true => {
                        let points_copy = points.clone();
                        model.colliders.push(Collider {
                            points: points_copy,
                        })
                    }
                    false => {
                        let points_copy = points.clone();
                        let mut accelerations = Vec::<Vec3A>::with_capacity(points.len());
                        for _ in 0..points.len() {
                            accelerations.push(Vec3A::ZERO);
                        }
                        model.curves.push(Curve {
                            points: points_copy,
                            accelerations,
                        })
                    }
                }
                model.total_points += points.len();
                points.clear();
            }
            if line.starts_with('#') {
                match line.split_at(2) {
                    (_, "Collider") => {
                        is_collider = true;
                    }
                    (_, "Origin") => {
                        is_collider = false;
                    }
                    (_, _) => {
                        println!("Unknown # sequence: {:?}", line)
                    }
                }
            }
        } else {
            let items: Vec<&str> = line.split(',').collect();
            if items.len() != 3 {
                println!("Error: line does not have 3 items: {:?}", line);
                continue;
            }

            let x: f32 = items[0].parse()?;
            let y: f32 = items[1].parse()?;
            let z: f32 = items[2].parse()?;

            points.push(Vec3A::new(x, y, z));
        }
    }

    let mut min_x = model.colliders[0].points[0].x;
    let mut max_x = model.colliders[0].points[0].x;
    let mut min_y = model.colliders[0].points[0].y;
    let mut max_y = model.colliders[0].points[0].y;
    for collider in model.colliders.iter() {
        for point in collider.points.iter() {
            if point.x < min_x {
                min_x = point.x;
            }
            if point.x > max_x {
                max_x = point.x;
            }
            if point.y < min_y {
                min_y = point.y;
            }
            if point.y > max_y {
                max_y = point.y;
            }
        }
    }

    model.bounding_box = Rect::from_corner_points([min_x, min_y], [max_x, max_y]);

    println!(
        "Model loaded [Origins: {:?}, Colliders: {:?}]",
        model.curves.len(),
        model.colliders.len()
    );
    Ok(model)
}

fn write_to_file(model: &Model) -> Result<()> {
    let file = File::create("./out.pp")?;
    let mut file = LineWriter::new(file);
    for curve in model.curves.iter() {
        file.write_all("# Curve\n".as_bytes())?;
        for point in curve.points.iter() {
            file.write_all(format!("{:?},{:?},{:?}\n", point.x, point.y, point.z).as_bytes())?;
        }
    }

    Ok(())
}
