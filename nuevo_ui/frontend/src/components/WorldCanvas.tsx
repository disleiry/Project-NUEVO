/**
 * WorldCanvas — live 2D world-frame map.
 *
 * Card shell and toggle legend styled to match OdometryCard (SensorSection).
 * Canvas style (translucent dark, subtle grid) is the original WorldCanvas design.
 */
import { useRef, useEffect, useState, useCallback } from 'react'
import { useRobotStore } from '../store/robotStore'

const PAD_MM          = 200
const LIDAR_THROTTLE_MS = 200

interface Trails { odom: boolean; gps: boolean; fused: boolean; lidar: boolean }

const SERIES = [
  { key: 'odom'  as const, label: 'Odometry', color: '#22d3ee' },
  { key: 'fused' as const, label: 'Fused',    color: '#4ade80' },
  { key: 'gps'   as const, label: 'GPS',      color: '#facc15' },
  { key: 'lidar' as const, label: 'Lidar',    color: '#f87171' },
]

export function WorldCanvas() {
  const canvasRef = useRef<HTMLCanvasElement>(null)

  const fusedPose   = useRobotStore((s) => s.fusedPose)
  const fusedTrail  = useRobotStore((s) => s.fusedPoseTrail)
  const odomTrail   = useRobotStore((s) => s.odometryTrail)
  const gpsStatus   = useRobotStore((s) => s.gpsStatus)
  const lidarPoints = useRobotStore((s) => s.lidarPoints)

  const scaleRef     = useRef<{ minX: number; maxX: number; minY: number; maxY: number } | null>(null)
  const lastLidarRef = useRef<number>(0)
  const [trails, setTrails] = useState<Trails>({ odom: true, gps: true, fused: true, lidar: true })

  const toggle = useCallback((key: keyof Trails) => {
    setTrails((t) => ({ ...t, [key]: !t[key] }))
  }, [])

  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    const dpr = window.devicePixelRatio || 1
    const W   = canvas.clientWidth
    const H   = canvas.clientHeight
    if (!W || !H) return
    canvas.width  = W * dpr
    canvas.height = H * dpr
    const ctx = canvas.getContext('2d')!
    ctx.scale(dpr, dpr)

    const allPts: [number, number][] = []
    if (trails.fused)  allPts.push(...fusedTrail)
    if (trails.odom)   allPts.push(...odomTrail)
    if (fusedPose)     allPts.push([fusedPose.x, fusedPose.y])
    if (trails.gps && gpsStatus?.is_detected) allPts.push([gpsStatus.x, gpsStatus.y])
    if (trails.lidar && lidarPoints) {
      for (let i = 0; i < lidarPoints.xs.length; i++)
        allPts.push([lidarPoints.xs[i], lidarPoints.ys[i]])
    }

    if (allPts.length > 0) {
      const xs = allPts.map((p) => p[0])
      const ys = allPts.map((p) => p[1])
      const next = {
        minX: Math.min(...xs) - PAD_MM, maxX: Math.max(...xs) + PAD_MM,
        minY: Math.min(...ys) - PAD_MM, maxY: Math.max(...ys) + PAD_MM,
      }
      scaleRef.current = scaleRef.current ? {
        minX: Math.min(scaleRef.current.minX, next.minX),
        maxX: Math.max(scaleRef.current.maxX, next.maxX),
        minY: Math.min(scaleRef.current.minY, next.minY),
        maxY: Math.max(scaleRef.current.maxY, next.maxY),
      } : next
    }

    const ext    = scaleRef.current ?? { minX: -500, maxX: 500, minY: -500, maxY: 500 }
    const rangeX = Math.max(ext.maxX - ext.minX, 1)
    const rangeY = Math.max(ext.maxY - ext.minY, 1)
    const scale  = Math.min(W / rangeX, H / rangeY)

    const toC = (wx: number, wy: number): [number, number] => [
      (wx - ext.minX) * scale,
      H - (wy - ext.minY) * scale,
    ]

    // Translucent dark background (original style)
    ctx.fillStyle = 'rgba(0,0,0,0.55)'
    ctx.fillRect(0, 0, W, H)

    // Grid — subtle white lines
    const gridMm = rangeX > 4000 ? 1000 : rangeX > 1000 ? 500 : 200
    ctx.strokeStyle = 'rgba(255,255,255,0.08)'
    ctx.lineWidth   = 0.5
    for (let gx = Math.floor(ext.minX / gridMm) * gridMm; gx <= ext.maxX; gx += gridMm) {
      const [cx] = toC(gx, 0)
      ctx.beginPath(); ctx.moveTo(cx, 0); ctx.lineTo(cx, H); ctx.stroke()
    }
    for (let gy = Math.floor(ext.minY / gridMm) * gridMm; gy <= ext.maxY; gy += gridMm) {
      const [, cy] = toC(0, gy)
      ctx.beginPath(); ctx.moveTo(0, cy); ctx.lineTo(W, cy); ctx.stroke()
    }

    // Origin cross
    const [ox, oy] = toC(0, 0)
    if (ox >= 0 && ox <= W && oy >= 0 && oy <= H) {
      ctx.strokeStyle = 'rgba(255,255,255,0.20)'
      ctx.lineWidth = 1
      ctx.beginPath(); ctx.moveTo(ox - 6, oy); ctx.lineTo(ox + 6, oy); ctx.stroke()
      ctx.beginPath(); ctx.moveTo(ox, oy - 6); ctx.lineTo(ox, oy + 6); ctx.stroke()
    }

    // Scale label
    const gridLabel = gridMm >= 1000 ? `${gridMm / 1000} m` : `${gridMm} mm`
    ctx.fillStyle = 'rgba(255,255,255,0.40)'
    ctx.font = '9px monospace'
    ctx.textAlign = 'right'
    ctx.textBaseline = 'bottom'
    ctx.fillText(`grid: ${gridLabel}`, W - 4, H - 3)

    // Odometry trail (cyan)
    if (trails.odom && odomTrail.length > 1) {
      ctx.beginPath()
      ctx.strokeStyle = 'rgba(34,211,238,0.55)'
      ctx.lineWidth = 1.5
      ctx.lineJoin = 'round'
      odomTrail.forEach(([wx, wy], i) => {
        const [cx, cy] = toC(wx, wy)
        i === 0 ? ctx.moveTo(cx, cy) : ctx.lineTo(cx, cy)
      })
      ctx.stroke()
    }

    // Fused trail (green)
    if (trails.fused && fusedTrail.length > 1) {
      ctx.beginPath()
      ctx.strokeStyle = 'rgba(74,222,128,0.70)'
      ctx.lineWidth = 1.5
      ctx.lineJoin = 'round'
      fusedTrail.forEach(([wx, wy], i) => {
        const [cx, cy] = toC(wx, wy)
        i === 0 ? ctx.moveTo(cx, cy) : ctx.lineTo(cx, cy)
      })
      ctx.stroke()
    }

    // GPS dot (yellow)
    if (trails.gps && gpsStatus?.is_detected) {
      const [cx, cy] = toC(gpsStatus.x, gpsStatus.y)
      ctx.beginPath()
      ctx.arc(cx, cy, 4, 0, Math.PI * 2)
      ctx.fillStyle = 'rgba(250,204,21,0.90)'
      ctx.fill()
    }

    // Lidar cloud (red, throttled)
    const now = Date.now()
    if (trails.lidar && lidarPoints && now - lastLidarRef.current >= LIDAR_THROTTLE_MS) {
      lastLidarRef.current = now
      ctx.fillStyle = 'rgba(248,113,113,0.60)'
      for (let i = 0; i < lidarPoints.xs.length; i++) {
        const [cx, cy] = toC(lidarPoints.xs[i], lidarPoints.ys[i])
        ctx.beginPath(); ctx.arc(cx, cy, 1.5, 0, Math.PI * 2); ctx.fill()
      }
    }

    // Robot at fused pose
    if (fusedPose) {
      const [rx, ry] = toC(fusedPose.x, fusedPose.y)
      const arrowLen = 14
      const hx = rx + arrowLen * Math.cos(fusedPose.theta)
      const hy = ry - arrowLen * Math.sin(fusedPose.theta)
      const nx = hx - rx, ny = hy - ry
      const len = Math.hypot(nx, ny) || 1
      const ux = nx / len, uy = ny / len
      const hw = 4

      ctx.beginPath()
      ctx.arc(rx, ry, 7, 0, Math.PI * 2)
      ctx.fillStyle   = 'rgba(255,255,255,0.90)'
      ctx.fill()

      ctx.strokeStyle = 'rgba(255,255,255,0.90)'
      ctx.lineWidth   = 2
      ctx.beginPath(); ctx.moveTo(rx, ry); ctx.lineTo(hx, hy); ctx.stroke()

      ctx.fillStyle = 'rgba(255,255,255,0.90)'
      ctx.beginPath()
      ctx.moveTo(hx, hy)
      ctx.lineTo(hx - ux * 8 + uy * hw, hy - uy * 8 - ux * hw)
      ctx.lineTo(hx - ux * 8 - uy * hw, hy - uy * 8 + ux * hw)
      ctx.closePath()
      ctx.fill()
    }
  }, [fusedPose, fusedTrail, odomTrail, gpsStatus, lidarPoints, trails])

  return (
    <div className="relative rounded-2xl p-4 backdrop-blur-2xl bg-white/10 border border-white/20 shadow-xl">
      <div className="absolute inset-x-0 top-0 h-px bg-gradient-to-r from-transparent via-white/50 to-transparent rounded-t-2xl" />
      <div className="absolute inset-0 rounded-2xl bg-gradient-to-br from-white/5 to-transparent opacity-50" />

      <div className="relative">
        <h4 className="text-sm font-semibold text-white mb-3">World Map</h4>

        {/* Series toggles — DCPlot legend style */}
        <div className="flex flex-wrap gap-x-3 gap-y-1 mb-2">
          {SERIES.map(({ key, label, color }) => (
            <button
              key={key}
              onClick={() => toggle(key)}
              className="flex items-center gap-1.5 cursor-pointer rounded px-1 py-0.5 transition-opacity hover:bg-white/10"
              style={{ opacity: trails[key] ? 1 : 0.35 }}
            >
              <span style={{ display: 'inline-block', width: 12, height: 3, borderRadius: 2, background: color }} />
              <span className="text-xs font-medium text-white/80">{label}</span>
            </button>
          ))}
        </div>

        <canvas
          ref={canvasRef}
          className="w-full rounded-xl"
          style={{ height: '420px' }}
        />
      </div>
    </div>
  )
}
